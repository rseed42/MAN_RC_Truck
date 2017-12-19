import pygame
from pygame.locals import *
import pygame.surfarray as surfarray
# ------------------------------------------------------------------------------
# App
# ------------------------------------------------------------------------------


class App:
    def __init__(self, configuration, image_sensor, controller, forward_distance_sensor, backward_distance_sensor):
        self.cfg = configuration
        self._running = True
        self._display_surf = None
        self.font = pygame.font.SysFont(self.cfg.app.surface.font, self.cfg.app.surface.font_size)
        self.clock = pygame.time.Clock()
        # Sensors
        self.image_sensor = image_sensor
        self.forward_distance_sensor = forward_distance_sensor
        self.backward_distance_sensor = backward_distance_sensor
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

    def show_text(self, msg, x, y, background_color=None):
        if not background_color:
            background_color = self.cfg.text.default.background_color
        text = self.font.render(msg, True, self.cfg.text.default.color, background_color)
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
        try:
            corners = self.image_sensor.corners
            aruco_center_coord = corners[0][0].mean(axis=0)[1]
            image_center_coord = (self.cfg.sensor.image.width / 2.)
            K_THRESH = 20
            #print("aruco_center", aruco_center_coord)
            #print("image_center", image_center_coord)
            if (aruco_center_coord - image_center_coord) < -K_THRESH:
                print("right")
                self.controller.turn_right()
            elif (aruco_center_coord - image_center_coord) > K_THRESH:
                print("left")
                self.controller.turn_left()
            else:
                print("straight")
                self.controller.straight()
        except:
            print("no code found -> dont move")

    def on_render(self):
        self._display_surf.fill(self.cfg.app.surface.screen_color)

        # Render the image and all information on this frame
        surfarray.blit_array(self._display_surf, self.image_sensor.last_image)

        self.show_text('FPS image: %.1f ' % self.image_sensor.average_framerate, 0, 0)
        self.show_text(' distance: %.1f ' % self.forward_distance_sensor.average_framerate, 140, 0)


        self.show_text('forward: {:06.2f} cm '.format(self.forward_distance_sensor.last_value), 0, 15, background_color=self.cfg.text.data.background_color)
        self.show_text('backward: {:06.2f} cm'.format(self.backward_distance_sensor.last_value), 170, 15, background_color=self.cfg.text.data.background_color)

        # Show all control values
        control_y = self._display_surf.get_height() - 15
        # Need to abbreviate on smaller displays
#        def show_signal((index, (name, channel))):
#            self.show_text('{}: {}'.format(name, channel.signal), index*110, control_y)
#        map(show_signal, enumerate(reversed(sorted(self.controller.channels.items()))))
        self.show_text('{}: {} '.format("trtl", self.controller.channels['throttle'].signal), 0, control_y)
        self.show_text('{}: {} '.format("steer", self.controller.channels['steering'].signal), 100, control_y)
        self.show_text('{}: {} '.format("shift", self.controller.channels['shift'].signal), 210, control_y)
        self.show_text('{}: {}'.format("leg", self.controller.channels['leg'].signal), 320, control_y)

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
