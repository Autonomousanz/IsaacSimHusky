# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
	
	

from omni.isaac.examples.base_sample import BaseSample
import numpy as np
from omni.isaac.core.objects import DynamicCuboid
### Robot ### 
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot

##### Camera 
from omni.isaac.sensor import Camera
import omni.isaac.core.utils.numpy.rotations as rot_utils
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from omni.isaac.core.utils.viewports import set_camera_view

#### Lidar
import omni                                                     # Provides the core omniverse apis
import asyncio                                                  # Used to run sample asynchronously to not block rendering thread
from omni.isaac.range_sensor import _range_sensor               # Imports the python bindings to interact with lidar sensor
from pxr import UsdGeom, Gf, UsdPhysics                         # pxr usd imports used to create the cube
import omni.isaac.RangeSensorSchema as RangeSensorSchema
from pxr import UsdGeom, UsdLux, Sdf, Gf, UsdPhysics
from omni.isaac.ui.ui_utils import setup_ui_headers, get_style, btn_builder, combo_cb_scrolling_frame_builder


### IMU sensor


import carb
import omni
import asyncio
import weakref
import omni.physx as _physx
import omni.ui as ui
from omni.isaac.sensor import _sensor
import omni.kit.commands
from pxr import Gf, UsdGeom

from omni.isaac.ui.ui_utils import setup_ui_headers, get_style, LABEL_WIDTH
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.ui.menu import make_menu_item_description
from omni.isaac.core.utils.nucleus import get_assets_root_path


class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()
        assets_root_path = get_assets_root_path()
        print(assets_root_path)
        if assets_root_path is None:
                # Use carb to log warnings, errors and infos in your application (shown on terminal)
                carb.log_error("Could not find nucleus server with /Isaac folder")
        asset_path = "/home/opencav-krovi/Desktop/test/test_husky3.usd"
        
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/husky")
        
        husky_robot = world.scene.add(Robot(prim_path="/World/.*/husky", name="husky"))

        ### Camera 

        HuskyCam = Camera(prim_path="/World/husky/husky/base_link/HuskyCam",
                            position=np.array([0.6, 0.08677, 0.65]),
                            frequency=20,
                            resolution=(256, 256),
                            orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),)
        camera = world.scene.add(HuskyCam)
        HuskyCam.initialize()
        HuskyCam.add_motion_vectors_to_frame()
        print(HuskyCam.get_current_frame())
        
        stage = omni.usd.get_context().get_stage()
        UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
        ### Lidar 
        self.lidarPath = "/World/Lidar"
        self.lidar = RangeSensorSchema.Lidar.Define(stage, Sdf.Path(self.lidarPath))
            # Horizontal and vertical field of view in degrees
        self.lidar.CreateHorizontalFovAttr().Set(360.0)
        self.lidar.CreateVerticalFovAttr().Set(10)

        # Rotation rate in Hz
        self.lidar.CreateRotationRateAttr().Set(20.0)

        # Horizontal and vertical resolution in degrees.  Rays will be fired on the bin boundries defined by the
        # resolution.  If your FOV is 45 degrees and your resolution is 15 degrees, you will get rays at
        # 0, 15, 30, and 45 degrees.
        self.lidar.CreateHorizontalResolutionAttr().Set(1.0)
        self.lidar.CreateVerticalResolutionAttr().Set(1.0)

        # Min and max range for the LIDAR.  This defines the starting and stopping locations for the linetrace
        self.lidar.CreateMinRangeAttr().Set(0.4)
        self.lidar.CreateMaxRangeAttr().Set(100.0)

        # These attributes affect drawing the lidar in the viewport.  High Level Of Detail (HighLod) = True will draw
        # all rays.  If false it will only draw horizontal rays.  Draw Lidar Points = True will draw the actual
        # LIDAR rays in the viewport.
        self.lidar.CreateHighLodAttr().Set(True)
        self.lidar.CreateDrawPointsAttr().Set(False)
        self.lidar.CreateDrawLinesAttr().Set(False)

        # We set the attributes we created.  We could have just set the attributes at creation, but this was
        # more illustrative.  It's important to remember that attributes do not exist until you create them; even
        # if they are defined in the schema.
        self.lidar.GetRotationRateAttr().Set(0.5)
        self.lidar.GetDrawLinesAttr().Set(True)
        self.lidar.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 0.250))

        # we want to make sure we can see the lidar we made, so we set the camera position and look target
        set_camera_view(eye=[5.00, 5.00, 5.00], target=[0.0, 0.0, 0.0], camera_prim_path="/OmniverseKit_Persp")


        #### IMU 
        self.body_path = "/World/husky/husky/imu_link"
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/sensor",
            parent=self.body_path,
            sensor_period=-1.0,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            visualize=True,
        )
        self._is = _sensor.acquire_imu_sensor_interface()
        self.sliders = []
        reading = self._is.get_sensor_readings(self.body_path + "/sensor")
        print(reading)
        if reading.shape[0]:
            self.sliders[0].model.set_value(float(reading[-1]["lin_acc_x"]) * self.meters_per_unit)  # readings
            self.sliders[1].model.set_value(float(reading[-1]["lin_acc_y"]) * self.meters_per_unit)  # readings
            self.sliders[2].model.set_value(float(reading[-1]["lin_acc_z"]) * self.meters_per_unit)  # readings
            self.sliders[3].model.set_value(float(reading[-1]["ang_vel_x"]))  # readings
            self.sliders[4].model.set_value(float(reading[-1]["ang_vel_y"]))  # readings
            self.sliders[5].model.set_value(float(reading[-1]["ang_vel_z"]))  # readings
            self.sliders[6].model.set_value(float(reading[-1]["orientation"][0]))  # readings
            self.sliders[7].model.set_value(float(reading[-1]["orientation"][1]))  # readings
            self.sliders[8].model.set_value(float(reading[-1]["orientation"][2]))  # readings
            self.sliders[9].model.set_value(float(reading[-1]["orientation"][3]))  # readings

        else:
            self.sliders[0].model.set_value(0)
            self.sliders[1].model.set_value(0)
            self.sliders[2].model.set_value(0)
            self.sliders[3].model.set_value(0)
            self.sliders[4].model.set_value(0)
            self.sliders[5].model.set_value(0)
            self.sliders[6].model.set_value(0)
            self.sliders[7].model.set_value(0)
            self.sliders[8].model.set_value(0)
            self.sliders[9].model.set_value(1)

        #$#####
        fancy_cube = world.scene.add(
            DynamicCuboid(
                prim_path="/World/random_cube",
                name="fancy_cube",
                position=np.array([2, 2, 1.0]),
                scale=np.array([0.5015, 0.5015, 0.5015]),
                color=np.array([0, 0, 1.0]),
            ))
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        self._cube = self._world.scene.get_object("fancy_cube")
        self._husky_robot = self._world.scene.get_object("husky")
        ####  Camera
        


        ### Lidar 
        maxDepth = self.lidar.GetMaxRangeAttr().Get()
        self._li = _range_sensor.acquire_lidar_sensor_interface()
        depth = self._li.get_depth_data(self.lidarPath)
        zenith = self._li.get_zenith_data(self.lidarPath)
        azimuth = self._li.get_azimuth_data(self.lidarPath)
        tableString = ""
        # self._info_cb, self._info_label = combo_cb_scrolling_frame_builder(**dict)
        # self._info_label.text = ""
        numCols = len(zenith)
        rowString = ""
        for i in range(numCols):
            rowString += "{" + str(i + 2) + ":." + str(5) + "f}   "
        rowString = "{0:16}  {1:10}" + rowString + "\n"

        tableString += rowString.format("Azimuth \ Zenith", " | ", *zenith)
        tableString += "-" * len(tableString) + "\n"
        for row, cols in enumerate(depth):
            # The data on the c++ side is stored as uint16.  in order to get our depth values into centimeters, we
            # must first convert from uint16 into float on [0,1], and then scale to the maximum distance.
            entry = [ray * maxDepth / 65535.0 for ray in cols]
            tableString += rowString.format("{0:.5f}".format(azimuth[row]), " | ", *entry)
        # self._info_label.text = tableString
        print(tableString)
        # self._HuskyCam = self._world.scene.get_object("HuskyCam")
        # print("cameraobject",self._HuskyCam)

        self._world.add_physics_callback("sim_step", callback_fn=self.robot_pose_info) #callback names have to be unique
        return

    # here we define the physics callback to be called before each physics step, all physics callbacks must take
    # step_size as an argument
    def print_cube_info(self, step_size):
        position, orientation = self._cube.get_world_pose()
        linear_velocity = self._cube.get_linear_velocity()
        # will be shown on terminal
        print("Cube position is : " + str(position))
        print("Cube's orientation is : " + str(orientation))
        print("Cube's linear velocity is : " + str(linear_velocity))
        return

    def robot_pose_info(self, step_size):
        pos, orient = self._husky_robot.get_world_pose()
        linear_vel = self._husky_robot.get_linear_velocity()
        # will be shown on terminal
        print("Robot position is : " + str(pos))
        print("Robot's orientation is : " + str(orient))
        print("Robot's linear velocity is : " + str(linear_vel))
        return


# vpi = omni.kit.viewport.get_viewport_interface()
# vpi.get_viewport_window().set_active_camera(str(camera_prim.GetPath()))