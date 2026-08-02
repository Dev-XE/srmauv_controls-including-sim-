import pygame
pygame.init()
pygame.joystick.init()
js = pygame.joystick.Joystick(0)
js.init()

while True:
    pygame.event.pump()
    vals = [round(js.get_axis(i), 2) for i in range(js.get_numaxes())]
    print(vals)