import sys
import pygame

pygame.init()

screen = pygame.display.set_mode((640, 480))
pygame.display.set_caption('keyboard ctrl')
screen.fill((0, 0, 0))

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()

    keys = pygame.key.get_pressed()
    for i in range(26):
        if keys[pygame.K_a + i]:
        #檢測A-Z
            print(chr(pygame.K_a + i))
    if keys[pygame.K_UP]:
    #上
        print("Up arrow")
    if keys[pygame.K_DOWN]:
    #下
        print("Down arrow")
    if keys[pygame.K_LEFT]:
    #左
        print("Left arrow")
    if keys[pygame.K_RIGHT]:
    #右
        print("Right arrow")
    if keys[pygame.K_ESCAPE]:
    #Esc退出
        pygame.quit()
        sys.exit()