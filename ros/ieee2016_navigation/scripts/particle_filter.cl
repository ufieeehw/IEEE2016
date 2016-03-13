typedef struct lidar
{   
    float4  coeff;
    float   min_distance;

} lidar;

float correct_distance(float dist, int offset, __global const lidar *l)
{
    float error;

    error = (l+offset)->coeff.s0 + 
            (l+offset)->coeff.s1 * dist +
            (l+offset)->coeff.s2 * pow(dist,2)+
            (l+offset)->coeff.s3 * pow(dist,3);

    return dist - error;
}

__kernel void trace( __global const float *p,  
                     __global const float *walls,
                     __global const lidar *l,
                     __global const int  *angle_to_lidar,
                              const uint  wall_cnt,
                     __global const float *angles_to_check,
                              const uint  angle_cnt,
                     __global const float *misfit_actual,
                     __global       float *weights) 
{

    int i = get_global_id(0);

    float3 ray_origin = (float3)(p[i*3], p[i*3+1],0);

    float3 t1,v1,v2;
    float v2_dot_v3;
    float t2,d,theta;
    float error = 0;
    float errors_measured = 0;

    // Change sigma here, less is tigher tolerance.
    float sigma2 = pow(.028,2);
    for(int a = 0; a < angle_cnt; a++){
        float min_dist = INFINITY;

        // Each compute unit will simulate a scan from one point. So we loop through every angle at
        // this point and set the current angle to theta.
        theta = p[i*3+2] + angles_to_check[a];
        float3 ray_direction = (float3)(cos(theta), sin(theta),0);

        // Now as part of the raytracing, we have to go through each wall and determine if we intersect.
        for(int w = 0; w < wall_cnt; w++){
            // This is a raytracing algorithm that finds intersections on a line segment.
            float3 point1 = (float3)(walls[5*w],walls[5*w+1],0);
            float3 point2 = (float3)(walls[5*w+2],walls[5*w+3],0);

            v1 = ray_origin - point1;
            v2 = point2 - point1;
            float3 v3 = (float3)(-ray_direction.s1, ray_direction.s0, 0);

            v2_dot_v3 = dot(v2,v3);
            
            if(v2_dot_v3 != 0){
                t1 = (float3)cross(v2, v1) / v2_dot_v3;
                t2 = dot(v1, v3) / v2_dot_v3;
                d = t1.s2;

                if(d >= 0 && t2 >= 0 && t2 <= 1){
                    // We intersected with a wall.
                    if (d < (l+angle_to_lidar[a])->min_distance){
                        // If we are too close to a wall, we need to move on to the next wall
                        float min_dist = INFINITY;
                        break;
                    }
                    // Now we have to check if it was a black wall and correct for that.
                    if(walls[5*w+4] == 1){
                        d = correct_distance(d, angle_to_lidar[a], l);
                    }
                    if(d < min_dist){
                        min_dist = d;
                    }
                }
            }
        }
        // Now that we have the smallest intersection, caclulate the error.
        if(min_dist != INFINITY){
            error += exp(-pow(misfit_actual[a]-min_dist,2)/(2*sigma2));
            errors_measured++;
        }
    }
    // Return the average error from each point.
    if (errors_measured == 0){
        weights[i] = 0;
    }else{
        weights[i] = error/errors_measured;    
    }
}
