# Perform Z Homing at specific XY coordinates.
#
# Copyright (C) 2019 Florian Heilmann <Florian.Heilmann@gmx.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
class SafeZHoming:
    def __init__(self, config):
        self.printer = config.get_printer()
        x_pos, y_pos = config.getfloatlist("home_xy_position", count=2)
        self.home_x_pos, self.home_y_pos = x_pos, y_pos
        self.z_hop = config.getfloat("z_hop", default=0.0)
        self.z_hop_speed = config.getfloat('z_hop_speed', 15., above=0.)
        self.speed = config.getfloat('speed', 50.0, above=0.)
        self.move_to_previous = config.getboolean('move_to_previous', False)
        self.printer.load_object(config, 'homing')
        self.gcode = self.printer.lookup_object('gcode')
        self.prev_G28 = self.gcode.register_command("G28", None)
        self.gcode.register_command("G28", self.cmd_G28)
        self.hooks = {}

        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        for axis in "xyz":
            name = 'before_'+ axis
            if config.get(name, None) is not None:
                self.hooks[name] = gcode_macro.load_template(config, name)

            name = 'after_'+ axis
            if config.get(name, None) is not None:
                self.hooks[name] = gcode_macro.load_template(config, name)

        if config.has_section("homing_override"):
            raise config.error("homing_override and safe_z_homing cannot"
                               +" be used simultaneously")

    def cmd_G28(self, gcmd):
        if self.z_hop != 0.0:
            self.do_z_hop()

        # Determine which axes we need to home
        need_x, need_y, need_z = [gcmd.get(axis, None) is not None
                                  for axis in "XYZ"]
        if not need_x and not need_y and not need_z:
            need_x = need_y = need_z = True

        if need_x:
            self.run_hook('before_x', gcmd)
            g28_gcmd = self.gcode.create_gcode_command("G28", "G28", {'X': 0})
            self.prev_G28(g28_gcmd)
            self.run_hook('after_x', gcmd)

        if need_y:
            self.run_hook('before_y', gcmd)
            g28_gcmd = self.gcode.create_gcode_command("G28", "G28", {'Y': 0})
            self.prev_G28(g28_gcmd)
            self.run_hook('after_y', gcmd)

        if need_z:
            self.run_hook('before_z', gcmd)
            self.home_z(gcmd)
            self.run_hook('after_z', gcmd)

    def run_hook(self, name, gcmd):
        if not name in self.hooks:
            return
        hook = self.hooks[name]
        context = hook.create_template_context()
        context['params'] = gcmd.get_command_parameters()
        hook.run_gcode_from_command(context)

    def do_z_hop(self):
        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.printer.get_reactor().monotonic()
        kin_status = toolhead.get_kinematics().get_status(curtime)
        pos = toolhead.get_position()

        if 'z' not in kin_status['homed_axes']:
            # Always perform the z_hop if the Z axis is not homed
            pos[2] = 0
            toolhead.set_position(pos, homing_axes=[2])
            toolhead.manual_move([None, None, self.z_hop], self.z_hop_speed)

            if hasattr(toolhead.get_kinematics(), "note_z_not_homed"):
                toolhead.get_kinematics().note_z_not_homed()

        elif pos[2] < self.z_hop:
            # If the Z axis is homed, and below z_hop, lift it to z_hop
            toolhead.manual_move([None, None, self.z_hop], self.z_hop_speed)

    def home_z(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')

        # Throw an error if X or Y are not homed
        curtime = self.printer.get_reactor().monotonic()
        kin_status = toolhead.get_kinematics().get_status(curtime)
        if ('x' not in kin_status['homed_axes'] or
            'y' not in kin_status['homed_axes']):
            raise gcmd.error("Must home X and Y axes first")
        # Move to safe XY homing position
        prevpos = toolhead.get_position()
        toolhead.manual_move([self.home_x_pos, self.home_y_pos], self.speed)
        # Home Z
        g28_gcmd = self.gcode.create_gcode_command("G28", "G28", {'Z': '0'})
        self.prev_G28(g28_gcmd)
        # Perform Z Hop again for pressure-based probes
        if self.z_hop:
            pos = toolhead.get_position()
            if pos[2] < self.z_hop:
                toolhead.manual_move([None, None, self.z_hop],
                                     self.z_hop_speed)
        # Move XY back to previous positions
        if self.move_to_previous:
            toolhead.manual_move(prevpos[:2], self.speed)

def load_config(config):
    return SafeZHoming(config)
