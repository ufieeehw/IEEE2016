#!/usr/bin/env python

import pygame
import sys

from time import time

red = (255,0,0)
green = (0,255,0)
blue = (0,0,255)
darkBlue = (0,0,128)
white = (255,255,255)
black = (0,0,0)
pink = (255,200,200)

DISPLAY_SIZE = (640,640)

ROBOT_SIZE = (50,100)
wheel_size = (5, 20)
movmnt_magnitube = 3


class robot(object):
	def __init__(self):
		self.ROBOT_SIZE = ROBOT_SIZE
		self.wheel_size = wheel_size
		self.robot_position = 0
		self.x_pos = 0
		self.y_pos = 0
		self.bigger = pygame.Rect(0, 0, 100, 50)
		self.rotatedsurf = pygame.Surface(ROBOT_SIZE)
		self.cur_theda = 0

	def move(self, destination):
		if destination[0] == 2: self.y_pos += movmnt_magnitube
		if destination[0] == 1: self.y_pos += -movmnt_magnitube
		if self.x_pos < 0: self.x_pos = 0
		if self.x_pos > DISPLAY_SIZE[0] - ROBOT_SIZE[0]: self.x_pos = DISPLAY_SIZE[0] - ROBOT_SIZE[0]
		if destination[1] == 1: self.x_pos += movmnt_magnitube
		if destination[1] == 2: self.x_pos += -movmnt_magnitube
		if self.y_pos < 0: self.y_pos = 0
		if self.y_pos > DISPLAY_SIZE[1]- ROBOT_SIZE[1]: self.y_pos = DISPLAY_SIZE[1] - ROBOT_SIZE[1]

	def rotate(self, rotation):
		if rotation[2] == 1: self.cur_theda += 1
		if self.cur_theda > 360: self.cur_theda = 0
		self.rotatedsurf = pygame.transform.rotate(surf, self.cur_theda )

	def draw(self):

		screen.blit(self.rotatedsurf, (self.x_pos,self.y_pos))
		'''
		pygame.draw.rect(surf, red, (self.x_pos, self.y_pos, self.ROBOT_SIZE[0], self.ROBOT_SIZE[1]), 0)
		pygame.draw.rect(screen, red, (self.x_pos, self.y_pos, self.ROBOT_SIZE[0], self.ROBOT_SIZE[1]), 0)
		pygame.draw.rect(screen, black, (self.x_pos +50, self.y_pos +10, self.wheel_size[0], self.wheel_size[1]), 0)
		pygame.draw.rect(screen, black, (self.x_pos +50, self.y_pos +65, self.wheel_size[0], self.wheel_size[1]), 0)
		pygame.draw.rect(screen, black, (self.x_pos -5, self.y_pos +10, self.wheel_size[0], self.wheel_size[1]), 0)
		pygame.draw.rect(screen, black, (self.x_pos -5, self.y_pos +65, self.wheel_size[0], self.wheel_size[1]), 0)
			'''

if __name__ == '__main__':
	pygame.init()
	pygame.key.set_repeat(1,10)
	screen = pygame.display.set_mode(DISPLAY_SIZE)
	surf =  pygame.Surface(ROBOT_SIZE)
	screen.fill(black)
	clock = pygame.time.Clock()
	bot = robot()

	while True:
		screen.fill(white)
		surf.fill(black)
		surf.set_colorkey((255, 0, 0))
		move_array = [0,0,0]

		for event in pygame.event.get():
			keys = pygame.key.get_pressed()
			if keys[pygame.K_i]: move_array[0] = 1
			if keys[pygame.K_k]: move_array[0] = 2
			if keys[pygame.K_l]: move_array[1] = 1
			if keys[pygame.K_j]: move_array[1] = 2
			if keys[pygame.K_n]: move_array[2] = 1

			if keys[pygame.K_q]: sys.exit()
			elif event.type == pygame.QUIT: sys.exit()

		bot.move(move_array)
		bot.rotate(move_array)
		bot.draw()
		clock.tick(60)
		pygame.display.flip()


