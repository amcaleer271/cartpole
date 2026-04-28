import pygame
import math
class visualizer:

    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((1280, 720))
        self.clock = pygame.time.Clock()
        self.running = True
        self.dt = 0

        #car pose
        self.x = 0.0
        self.theta = 0.0
        self.cart_pos = pygame.Vector2(self.screen.get_width() / 2, 600)
        self.pendulum_pos = pygame.Vector2(self.screen.get_width() / 2, self.screen.get_height() / 2)

        self.cart_width = 100.0

        #multiply positions to make them easier to see
        self.multiplier = 1.0
        
    def update(self, x, theta, L, u):
        if self.running:
            self.x = x
            self.theta = theta
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            self.screen.fill("white")
            self.cart_pos.x = (60*self.x) + self.screen.get_width() / 2
            self.pendulum_pos.x = self.cart_pos.x + (self.cart_width/2)- 60 * math.sin(self.theta)
            self.pendulum_pos.y = self.cart_pos.y - 60 * math.cos(self.theta)

            #Draw the pendulum and cart
            pygame.draw.rect(self.screen, "red", (self.cart_pos.x, self.cart_pos.y, self.cart_width, 50))
            pygame.draw.circle(self.screen, "orange", self.pendulum_pos, 10)
            pygame.draw.line(self.screen, "blue", (self.cart_pos.x + self.cart_width/2, self.cart_pos.y), self.pendulum_pos, width=2)

            #Draw the applied force u
            pygame.draw.line(self.screen, "green", (self.cart_pos.x + self.cart_width/2, self.cart_pos.y+25), (self.cart_pos.x + self.cart_width/2 + (3*u), self.cart_pos.y+25),width=5)
            pygame.display.flip()
            dt = self.clock.tick(60)/1000

    def end(self):
        self.running = False
        pygame.quit()
