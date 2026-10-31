import pygame
pygame.init()
pygame.joystick.init()

joysticks = []
for i in range(pygame.joystick.get_count()):
    controller = pygame.joystick.Joystick(i)
    controller.init()
    joysticks.append(controller)
    print(f"Detected controller: {controller.get_name()}")

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.JOYBUTTONDOWN:
            print(f"Button {event.button} pressed on controller {event.joy}")
        elif event.type == pygame.JOYAXISMOTION:
            print(f"Axis {event.axis} moved to {event.value} on controller {event.joy}")
        elif event.type == pygame.JOYHATMOTION:
            print(f"Hat {event.hat} moved to {event.value} on controller {event.joy}")