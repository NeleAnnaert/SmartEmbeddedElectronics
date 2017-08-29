from . import GUI
from . import Mapper, Location


class GUIStandalone(GUI):
    def __init__(self, send_mapper, **kwargs):
        self.send_mapper = send_mapper

        def send_motor(*args, **kwargs):
            print("send motor")
            print("args")
            for count, thing in enumerate(args):
                print('{0}. {1}'.format(count, thing))
            print("kwargs")
            for name, value in kwargs.items():
                print('{0} = {1}'.format(name, value))

        width = 159
        height = 96
        self.mapper = Mapper(width, height, send_motor)
        super().__init__(self.mapper, **kwargs)

    def update_mapper(self, map_matrix, current_loc, previous_loc, parking_loc, previous_park, weight,
                      send_command):
        self.mapper.map_matrix = map_matrix
        self.mapper.current_loc = current_loc
        self.mapper.previous_loc = previous_loc
        self.mapper.parking_loc = parking_loc
        self.mapper.previous_park = previous_park
        self.mapper.weight = weight
        self.mapper.send_command = send_command
        self.mapper.path_plan()
        self.grid_kleur_in()

    def update_item(self):
        changes = super().update_item()
        if changes:
            self.send_mapper(self.mapper)

    def paint_modus(self):
        super().paint_modus()
        if not self.in_paint_modus:
            self.send_mapper(self.mapper)

    def send_command(self):
        super().send_command()
self.send_mapper(self.mapper)
