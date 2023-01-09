from Geometry3D import *
from . import probe
import asyncio
import logging

# ref: https://docs.sympy.org/latest/modules/geometry/plane.html
class AutoZPositions:
  def __init__(self, config):
    # use ProbePointsHelper to validate config
    helper = probe.ProbePointsHelper(config, lambda *args: None)
    helper.minimum_points(3)

    self.printer = config.get_printer()
    self.config = config
    # points = config.get('points', None)
    self.gcode = self.printer.lookup_object('gcode')
    self.z_steppers = []

    # TODO(ibash) add helper descriptions for commands
    self.gcode.register_command('AUTO_Z_POSITIONS', self.cmd_AUTO_Z_POSITIONS)
    self.gcode.register_command('PROBE_PLANE', self.cmd_PROBE_PLANE)
    self.printer.register_event_handler("klippy:connect", self.handle_connect)

  def handle_connect(self):
      kin = self.printer.lookup_object('toolhead').get_kinematics()
      z_steppers = [s for s in kin.get_steppers() if s.is_active_axis('z')]
      if len(z_steppers) != 3:
          raise self.printer.config_error("%s needs exactly 3 z steppers" % self.config.get_name())
      self.z_steppers = z_steppers

  def cmd_AUTO_Z_POSITIONS(self, gcmd):
    z_positions = asyncio.run(self.auto_z_positions(gcmd))
    gcmd.respond_info("z positions: %s" % z_positions)

  def cmd_PROBE_PLANE(self, gcmd):
    plane = asyncio.run(self.probe_plane(gcmd))
    gcmd.respond_info("probed plane: %s" % plane)

  async def auto_z_positions(self, gcmd):
    # algorithm
    # 1. probe a base plane (plane 1)
    # 2. for each z stepper:
    #   1. move the z stepper
    #   2. probe a plane (planes 2-4)
    #   3. move the z stepper back
    # 3. planes 2-4 intersected with the base plane (plane 1) produce a line
    # 4. each line intersected with the other produce a pivot point
    base = await self.probe_plane(gcmd)
    # steppers = ["stepper_z", "stepper_z1", "stepper_z2"]
    distance = 3
    planes = [base]
    for stepper in self.z_steppers:
      # TODO(ibash) how to wait for this gcode to finish...?
      self.move_z_stepper(stepper, distance)
      plane = await self.probe_plane(gcmd)
      planes.append(plane)
      self.move_z_stepper(stepper, -1 * distance)

    z_positions = self.compute_z_positions(planes)
    return z_positions

  async def probe_plane(self, gcmd):
    result = await self.probe_async(gcmd)
    points = result["points"]
    plane = Plane(points[0], points[1], points[2])
    return plane

  def probe_async(self, gcmd):
    future = asyncio.Future()
    def finalize(offsets, positions):
      # TODO(ibash) should the x and y here be adjusted by offsets?
      points = [ Point(x, y, z) for (x, y, z) in positions]
      future.set_result({
        "offsets": offsets,
        "points": points,
        "positions": positions,
      })

    helper = probe.ProbePointsHelper(self.config, finalize)
    helper.minimum_points(3)
    helper.start_probe(gcmd)

    return future

  def move_z_stepper(self, stepper, distance):
    # "FORCE_MOVE STEPPER=<config_name> DISTANCE=<value> VELOCITY=<value> [ACCEL=<value>]"
    # self.gcode.run_script_from_command(
    #     "M204 S%.3f" % (self.probe_accel,))

    # TODO(ibash) stepper = "stepper_z", "stepper_z1", "stepper_z2"
    # TODO(ibash) get speed from probe lift_speed
    speed = 3

    stepper_name = stepper.get_name()
    # distance = 10
    self.gcode.run_script_from_command(
        "FORCE_MOVE STEPPER=%s DISTANCE=%.3f VELOCITY=%.3f"  % (stepper_name, distance, speed))

  def compute_z_positions(self, planes):
    lines = [
      planes[0].intersection(planes[1]),
      planes[0].intersection(planes[2]),
      planes[0].intersection(planes[3])
    ]

    points = [
      lines[0].intersection(lines[1]),
      lines[0].intersection(lines[2]),
      lines[1].intersection(lines[2])
    ]

    return points


def load_config(config):
  return AutoZPositions(config)

# class AutoZPositions:
# 
#   def __init__(self, config):
#     logging.info("BLAH")
#     gcode = config.get_printer().lookup_object('gcode')
#     gcode.register_command('AUTO_Z_POSITIONS', self.cmd_AUTO_Z_POSITIONS)
#     gcode.register_command('PROBE_PLANE', self.cmd_PROBE_PLANE)
#     points = config.get('points', None)
#     logging.info("Config points: %s", points)
# 
#     # TODO(ibash) validate that there are 3 z steppers
# 
#   def cmd_AUTO_Z_POSITIONS(self, gcmd):
#     asyncio.run(self.auto_z_positions(gcmd))
# 
#   def cmd_PROBE_PLANE(self, gcmd):
#     asyncio.run(self.probe_plane_2(gcmd))
# 
#   async def probe_plane_2(self, gcmd):
#     base = await self.probe_plane(gcmd)
#     logging.info("Probed a plane: %s", base)
# 
#   async def auto_z_positions(self, gcmd):
# 
#     # TODO(ibash): calculate the z positions for z_tilt
# 
#     # TODO(ibash) algorithm
#     # 1. Probe 3 points
#     # 2. Get plane P1 from those points
#     # 3. Move one z stepper
#     # 4. Probe 3 points
#     # 5. Get plane P2 from those points
#     # 6. Reset position of z stepper
#     # 7. Repeat for other z steppers toget planes P3-P6, for a total of six planes
#     # 8. P1 intersected with P2 gives a line between the second and third
#     #    stepper; same for P3 intersected with P4, and P5, P6.
#     # 9. Call each line, L1, L2, L3
#     # 10. The intersect of each pair of lines is a pivot point / z position
# 
#     # TODO(ibash) algorithm revised:
#     # probe 3 points for a base plane
#     #
#     # for each z stepper:
#     #   move the z stepper up by offset
#     #   probe 3 points for a plane
#     #   move the z stepper down by offset
#     #
#     # intersect each plane after perturbation to get a line
#     # intersect all lines to get the points
#     base = await self.probe_plane(gcmd)
#     # TODO(ibash) get steppers from config
#     steppers = ["stepper_z", "stepper_z1", "stepper_z2"]
#     distance = 5
#     planes = []
#     for stepper in steppers:
#       # TODO(ibash) how to wait for this gcode to finish...?
#       self.move_z_stepper(stepper, distance)
#       plane = await self.probe_plane(gcmd)
#       planes.append(plane)
#       self.move_z_stepper(stepper, -1 * distance)
# 
#     z_positions = self.compute_z_positions(planes)
# 
# 
#   async def probe_plane(self, gcmd):
#     result = await self.probe_async(gcmd)
#     points = results["points"]
#     plane = Plane(points[0], points[1], points[2])
#     return plane
# 
#   def probe_async(self):
#     future = asyncio.Future()
#     def finalize(offsets, positions):
#       # TODO(ibash) should the x and y here be adjusted by offsets?
#       points = [ Point(x, y, z) for (x, y), z in zip(positions, positions)]
# 
#       # TODO(ibash) remove offsets?
#       future.set_result({
#         "offsets": offsets,
#         "points": points,
#         "positions": positions,
#       })
# 
#     helper = probe.ProbePointsHelper(config, finalize)
#     helper.minimum_points(2)
#     helper.start_probe(gcmd)
# 
#     return future
# 
#   def move_z_stepper(self, stepper, distance):
#     # "FORCE_MOVE STEPPER=<config_name> DISTANCE=<value> VELOCITY=<value> [ACCEL=<value>]"
#     # self.gcode.run_script_from_command(
#     #     "M204 S%.3f" % (self.probe_accel,))
# 
#     # TODO(ibash) stepper = "stepper_z", "stepper_z1", "stepper_z2"
#     # TODO(ibash) velocity
#     speed = self.probe_helper.get_lift_speed()
#     # distance = 10
#     self.gcode.run_script_from_command(
#         "FORCE_MOVE STEPPER=%s DISTANCE=%.3f VELOCITY=%.3f"  % (stepper, distance, speed))
# 
#   def compute_z_positions(self, planes):
#     pass
# 
#     # lines = []
#     # len = planes.length / 2
#     # for (var i = 0; i < len; i++) {
#     #     a = planes[(i * 2)]
#     #     b = planes[(i * 2) + 1]
#     #     lines.append(a.intersection(b))
#     # }
# 
#     # z_positions = []
#     # a = line[0].intersection(line[1])
#     # b = line[0].intersection(line[2])
#     # c = line[1].intersection(line[2])
# 
#     # # TODO(ibash) return the result / log it to console
# 
#     # return [a, b, c]
# 
# def load_config(config):
 
#   return AutoZPositions(config)
