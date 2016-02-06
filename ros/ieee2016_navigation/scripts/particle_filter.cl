__kernel void trace( __global const float *p,  
                     __global const float *walls,
                              const uint  wall_cnt,
                     __global const float *angles_to_check,
                              const uint  angle_cnt,
                     __global const float *misfit_actual,
                     __global       float *weights
    ) {

    int i = get_global_id(0);

    __local float error;
    __local float min;
    __local float sigma2;
    __local float errors_measured;

    float3 ray_origin = (float3)(p[i*3], p[i*3+1],0);

    float3 t1,v1,v2;
    float v2_dot_v3;
    float t2,d,theta;
    //float temp;
    //temp = p[0];
    error = 0;
    errors_measured = 0;

    // Change tolerance here
    sigma2 = pow(.5,2);
    for(int a = 0; a < angle_cnt; a++){
        min = 1000;
        theta = p[i*3+2] + angles_to_check[a];
        float3 ray_direction = (float3)(cos(theta), sin(theta),0);

        for(int w = 0; w < wall_cnt; w++){
            float3 point1 = (float3)(walls[4*w],walls[4*w+1],0);
            float3 point2 = (float3)(walls[4*w+2],walls[4*w+3],0);

            v1 = ray_origin - point1;
            v2 = point2 - point1;
            float3 v3 = (float3)(-ray_direction.s1, ray_direction.s0, 0);

            v2_dot_v3 = dot(v2,v3);
            if(v2_dot_v3 != 0){
                t1 = cross(v2, v1) / v2_dot_v3;
                t2 = dot(v1, v3) / v2_dot_v3;
                d = t1.s2;
                if(d >= 0.0 && t2 >= 0.0 && t2 <= 1.0){
                    if(d < min){
                        min = d;
                    }
                }
            }
        }
        if(min != 1000){
            error += exp(-pow(misfit_actual[a]-min,2)/(2*sigma2));
            errors_measured++;
        }
    }
    if (errors_measured == 0){
        weights[i] = 0;
    }else{
        weights[i] = error/errors_measured;    
    }
    
}