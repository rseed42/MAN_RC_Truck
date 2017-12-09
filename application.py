import pygame
from pygame.locals import *
import pygame.surfarray as surfarray
# ------------------------------------------------------------------------------
# App
# ------------------------------------------------------------------------------


class App:
    def __init__(self, configuration, image_sensor, controller):
        self.cfg = configuration
        self._running = True
        self._display_surf = None
        self.font = pygame.font.SysFont(self.cfg.app.surface.font, self.cfg.app.surface.font_size)
        self.clock = pygame.time.Clock()
        # Sensors
        self.image_sensor = image_sensor
        self.controller = controller

    def on_init(self):
        # Initialize the primary display surface
        self._display_surf = pygame.display.set_mode(
            (self.cfg.app.surface.width, self.cfg.app.surface.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF
        )
        self._running = True
        # Return true if everything went well, otherwise we have to abort the application
        return True

    def show_text(self, msg, x, y):
        text = self.font.render(msg, True, self.cfg.text.default.color, self.cfg.text.default.background_color)
        self._display_surf.blit(text, (x, y))

    def on_event(self, event):
        if event.type == pygame.KEYUP:
            self.controller.on_key_up(event)

        if event.type == pygame.KEYDOWN:
            if event.key == K_q:
                self._running = False
            self.controller.on_key_down(event)

        if event.type == pygame.QUIT:
            self._running = False

    def on_key_pressed(self, keys):
        self.controller.on_key_pressed(keys)

    def on_loop(self):
        pass

    def on_render(self):
        self._display_surf.fill(self.cfg.app.surface.screen_color)

        # Render the image and all information on this frame
        surfarray.blit_array(self._display_surf, self.image_sensor.last_image)

        self.show_text('image fps: %.1f' % self.image_sensor.average_framerate, 0, 0)

        # Show all control values
        def show_signal((index, (name, channel))):
            self.show_text('{}: {}'.format(name, channel.signal), 0, 30 + 15*index)
        map(show_signal, enumerate(self.controller.channels.items()))

        # Update the surface after we have applied all drawing operations
        pygame.display.flip()
        # Fixed fps rate for the application
        self.clock.tick(self.cfg.app.clock_rate)

    def on_cleanup(self):
        pygame.quit()

    def on_execute(self):
        # If we can not initialize the application, quit
        if not self.on_init():
            self._running = False
        while self._running:
            self.on_key_pressed(pygame.key.get_pressed())
            for event in pygame.event.get():
                self.on_event(event)
            self.on_loop()
            self.on_render()
        self.on_cleanup()
