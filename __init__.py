try:
    from .firmware.pc_interface.dxlgripper_interface import DxlGripperInterface

except ImportError:
    print("Error importing online modules in YetAnotherRobotGripper. Skipping.")