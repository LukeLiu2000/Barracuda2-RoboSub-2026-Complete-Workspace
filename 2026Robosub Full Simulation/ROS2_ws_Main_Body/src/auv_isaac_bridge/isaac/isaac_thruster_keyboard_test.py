# -*- coding: utf-8 -*-
"""Isaac Sim Script Editor: hold 1..8 -> thruster_0..7 local +X.
Paste the ENTIRE file and Run; then Play and click the viewport.
Space/0: latch stop; R: re-enable; Q: uninstall this test.
Release stops applying thrust, NOT the body's existing velocity.
No ROS, MPPI, buoyancy, damping or scene-file edits are performed.
Full thrust uses the project's configured positive limit, 39.2266 N.
"""
import builtins
import math
import carb
import carb.input
import omni.appwindow
import omni.physx
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics, UsdUtils, PhysicsSchemaTools

# Change these only if the Stage hierarchy is different.
BODY_PATH = "/World/Barracuda_2/barracuda2/base_link"
THRUSTER_PATHS = ["/World/Barracuda_2/barracuda2/thruster_%d" % i for i in range(8)]
FULL_THRUST_N = 39.2266
# Automatically stop the known project bridge to prevent superimposed forces.
STOP_EXISTING_PROJECT_BRIDGE = True


class DirectThrusterKeyboardTest:
    def __init__(self):
        self.alive = True
        self.enabled = True
        self.held = set()
        self.key_sub = None
        self.physics_sub = None
        self.timeline_sub = None
        self.stage_sub = None
        self.stage = omni.usd.get_context().get_stage()
        if self.stage is None:
            raise RuntimeError("No Stage open. Load the robot scene first.")
        self.body = self.stage.GetPrimAtPath(BODY_PATH)
        if not self.body.IsValid() or not self.body.HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError("BODY_PATH is not a rigid body: " + BODY_PATH)
        rb = UsdPhysics.RigidBodyAPI(self.body)
        if rb.GetKinematicEnabledAttr().Get():
            raise RuntimeError("Body is kinematic; select the dynamic robot rigid body.")
        if rb.GetRigidBodyEnabledAttr().Get() is False:
            raise RuntimeError("Rigid body is disabled.")
        self.thrusters = [self.stage.GetPrimAtPath(p) for p in THRUSTER_PATHS]
        for p, prim in zip(THRUSTER_PATHS, self.thrusters):
            if not prim.IsValid() or not prim.IsA(UsdGeom.Xformable):
                raise RuntimeError("Missing/non-transformable thruster: " + p)
        self.mpu = float(UsdGeom.GetStageMetersPerUnit(self.stage))
        if not math.isfinite(self.mpu) or self.mpu <= 0:
            raise RuntimeError("Invalid metersPerUnit")
        # USD force units are massUnits * lengthUnits / second^2.
        get_kpu = getattr(UsdPhysics, "GetStageKilogramsPerUnit", None)
        self.kpu = float(get_kpu(self.stage)) if get_kpu else 1.0
        if not math.isfinite(self.kpu) or self.kpu <= 0:
            raise RuntimeError("Invalid kilogramsPerUnit")
        self.force_numeric = FULL_THRUST_N / (self.mpu * self.kpu)
        self.stage_id = int(UsdUtils.StageCache.Get().GetId(self.stage).ToLongInt())
        if self.stage_id <= 0:
            raise RuntimeError("Stage is not registered in USD StageCache")
        self.body_id = PhysicsSchemaTools.sdfPathToInt(BODY_PATH)
        self.sim = omni.physx.get_physx_simulation_interface()
        self.timeline = omni.timeline.get_timeline_interface()
        app_window = omni.appwindow.get_default_app_window()
        if app_window is None:
            raise RuntimeError("No Isaac application window/keyboard available")
        self.keyboard = app_window.get_keyboard()
        if self.keyboard is None:
            raise RuntimeError("No application keyboard available")
        self.input = carb.input.acquire_input_interface()
        self.keymap = {getattr(carb.input.KeyboardInput, "KEY_%d" % (i + 1)): i
                       for i in range(8)}
        if STOP_EXISTING_PROJECT_BRIDGE:
            bridge = getattr(builtins, "_AUV_ISAAC_BRIDGE_INSTANCE", None)
            if bridge is not None:
                bridge.shutdown()
                builtins._AUV_ISAAC_BRIDGE_INSTANCE = None
                print("[thruster-test] Existing project bridge stopped; damping also stopped.")
        self.key_sub = self.input.subscribe_to_keyboard_events(self.keyboard, self.on_key)
        self.timeline_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop(
            self.on_timeline, name="direct_thruster_timeline")
        self.stage_sub = omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(
            self.on_stage, name="direct_thruster_stage")
        self.physics_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self.on_step)
        print("[thruster-test] READY: hold 1..8 = t0..t7 local +X, %.4f N each" % FULL_THRUST_N)
        print("[thruster-test] Space/0 STOP; R re-arm; Q uninstall. Click viewport, then press keys.")
        print("[thruster-test] body=%s; m/unit=%g; kg/unit=%g" % (BODY_PATH, self.mpu, self.kpu))
        for i, prim in enumerate(self.thrusters):
            pos, direction = self.world_axis(prim)
            print("  key %d -> t%d: world +X direction=%s, point=%s" %
                  (i + 1, i, tuple(round(float(x), 5) for x in direction), tuple(pos)))

    @staticmethod
    def world_axis(prim):
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(UsdGeomTimeDefault)
        # TransformDir ignores translation and includes ancestor transforms.
        # Normalize to remove axis scale: force magnitude remains FULL_THRUST_N.
        axis = matrix.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))
        length = axis.GetLength()
        if not math.isfinite(length) or length < 1e-12:
            raise RuntimeError("Degenerate local X axis: " + str(prim.GetPath()))
        return matrix.ExtractTranslation(), axis / length

    def clear(self, reason):
        self.held.clear()
        print("[thruster-test] " + reason)

    def on_key(self, event, *args):
        try:
            if not self.alive:
                return True
            typ = event.type
            key = event.input
            press = typ == carb.input.KeyboardEventType.KEY_PRESS
            repeat = typ == carb.input.KeyboardEventType.KEY_REPEAT
            release = typ == carb.input.KeyboardEventType.KEY_RELEASE
            K = carb.input.KeyboardInput
            if press and key in (K.SPACE, K.KEY_0):
                self.enabled = False
                self.clear("STOP latched; R to re-arm")
            elif press and key == K.R:
                self.clear("Re-armed; release and press a number key")
                self.enabled = True
            elif press and key == K.Q:
                self.shutdown()
            elif key in self.keymap:
                index = self.keymap[key]
                if release:
                    if index in self.held:
                        self.held.discard(index)
                        print("[thruster-test] t%d OFF" % index)
                elif press and self.enabled and self.timeline.is_playing():
                    self.held.add(index)
                    print("[thruster-test] t%d ON: +local X, %.4f N" % (index, FULL_THRUST_N))
                # Repeat does not re-arm after pause/stop; a new press is required.
            return True
        except Exception as exc:
            self.enabled = False
            self.clear("Keyboard error: " + str(exc))
            return True

    def on_timeline(self, event):
        if int(event.type) in (int(omni.timeline.TimelineEventType.STOP),
                               int(omni.timeline.TimelineEventType.PAUSE),
                               int(omni.timeline.TimelineEventType.PLAY)):
            self.held.clear()

    def on_stage(self, event):
        if int(event.type) in (int(omni.usd.StageEventType.CLOSING),
                               int(omni.usd.StageEventType.OPENED)):
            self.shutdown()

    def on_step(self, dt):
        if not self.alive or not self.enabled or not self.timeline.is_playing():
            return
        try:
            if omni.usd.get_context().get_stage() != self.stage:
                raise RuntimeError("Stage changed; run script again")
            for i in tuple(self.held):
                # Poll also to clear released keys if an event was missed.
                key = getattr(carb.input.KeyboardInput, "KEY_%d" % (i + 1))
                if self.input.get_keyboard_value(self.keyboard, key) < 0.5:
                    self.held.discard(i)
                    continue
                pos, axis = self.world_axis(self.thrusters[i])
                f = axis * self.force_numeric
                self.sim.apply_force_at_pos(
                    self.stage_id, self.body_id,
                    carb.Float3(float(f[0]), float(f[1]), float(f[2])),
                    carb.Float3(float(pos[0]), float(pos[1]), float(pos[2])))
        except Exception as exc:
            self.enabled = False
            self.clear("Physics error (test disabled): " + str(exc))
            carb.log_error("[thruster-test] " + str(exc))

    def shutdown(self):
        self.alive = False
        self.held.clear()
        if self.key_sub is not None:
            self.input.unsubscribe_to_keyboard_events(self.keyboard, self.key_sub)
            self.key_sub = None
        self.physics_sub = None
        self.timeline_sub = None
        self.stage_sub = None
        print("[thruster-test] Uninstalled. No more forces from this script.")


# Explicit USD default time avoids accidentally passing a float/time sample.
from pxr import Usd
UsdGeomTimeDefault = Usd.TimeCode.Default()
_previous = getattr(builtins, "_AUV_DIRECT_THRUSTER_TEST", None)
if _previous is not None:
    _previous.shutdown()
builtins._AUV_DIRECT_THRUSTER_TEST = DirectThrusterKeyboardTest()