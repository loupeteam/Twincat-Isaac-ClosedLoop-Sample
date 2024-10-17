import carb
from omni.kit.scripting import BehaviorScript
from loupe.simulation.beckhoff_bridge import BeckhoffBridge
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction 
from omni.isaac.core.prims import RigidPrimView, RigidPrim

actPos = 0.0
setPos = 0.0
setVel = 0.0
# This function gets called once on init, and should be used to subscribe to cyclic reads.
def on_beckoff_init(event):
    # Create a list of variable names to be read cyclically, and add to Manager
    beckhoff_bridge.add_cyclic_read_variables(["MAIN.ax.NcToPlc.SetPos", "MAIN.ax.NcToPlc.SetVelo"])


# This function is called every time the bridge receives new data
def on_message(event):
    try:
        global setPos
        global setVel
        setPos = event.payload["data"]["MAIN"]["ax"]["NcToPlc"]["SetPos"]
        setVel = event.payload["data"]["MAIN"]["ax"]["NcToPlc"]["SetVelo"]
        # setVel = event.payload["data"]["MotionSystems"]["Joint"]["Position"]

        # setVel = event.payload["data"]["RAPID"]["MODULE1"]["PLC"]["SetVelo"]



#        carb.log_info(f"q_ui1: {q_ui1}, q_ui2: {q_ui2}")
    except KeyError as e:
        pass
        # carb.log_error(f"KeyError: {e}")

# Instantiate the bridge and register lifecycle subscriptions
beckhoff_bridge = BeckhoffBridge.Manager("PLC1")
beckhoff_bridge.register_init_callback(on_beckoff_init)
beckhoff_bridge.register_data_callback(on_message)

articulation = Articulation("/World/Robot")

class Box(BehaviorScript):        
    def on_init(self):
        carb.log_info(f"{type(self).__name__}.on_init()->{self.prim_path}")

    def on_destroy(self):
        carb.log_info(f"{type(self).__name__}.on_destroy()->{self.prim_path}")

    def on_play(self):
        carb.log_info(f"{type(self).__name__}.on_play()->{self.prim_path}")

    def on_pause(self):
        carb.log_info(f"{type(self).__name__}.on_pause()->{self.prim_path}")

    def on_stop(self):
        carb.log_info(f"{type(self).__name__}.on_stop()->{self.prim_path}")

    def on_update(self, current_time: float, delta_time: float):

        if not articulation.handles_initialized:
            articulation.initialize()

        velocity = [setVel/100]
        positions = [setPos/100]

        # articulation.apply_action(control_actions=ArticulationAction(joint_positions=positions, joint_velocities=velocity)) 

        joint_positions = articulation.get_joint_positions()

        global actPos
        actPos = float(joint_positions[0])*100

        beckhoff_bridge.write_variable(
            "MAIN.actPos", actPos
        )
