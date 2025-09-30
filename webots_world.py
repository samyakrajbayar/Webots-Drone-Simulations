#VRML_SIM R2023b utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/floors/protos/RectangleArena.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/buildings/protos/SimpleBuilding.proto"

WorldInfo {
  info [
    "Quadrotor Autopilot Simulation with Wind"
  ]
  title "Drone Autopilot with Wind Effects"
  basicTimeStep 8
  FPS 15
}
Viewpoint {
  orientation -0.2 0.9 0.35 1.2
  position -5 -15 12
}
TexturedBackground {
}
TexturedBackgroundLight {
}
RectangleArena {
  floorSize 50 50
  floorTileSize 2 2
}

# Buildings/Structures
SimpleBuilding {
  translation 10 10 0
  corners [
    0 0
    0 8
    8 8
    8 0
  ]
  wallHeight 6
}
SimpleBuilding {
  translation -15 5 0
  corners [
    0 0
    0 6
    6 6
    6 0
  ]
  wallHeight 8
}
SimpleBuilding {
  translation 5 -12 0
  corners [
    0 0
    0 5
    5 5
    5 0
  ]
  wallHeight 4
}

# Quadrotor Robot
Robot {
  translation 0 0 1
  rotation 0 0 1 0
  children [
    # Central body
    Shape {
      appearance PBRAppearance {
        baseColor 0.1 0.1 0.1
        metalness 0.7
      }
      geometry Box {
        size 0.1 0.1 0.05
      }
    }
    
    # Propellers with motors
    DEF MOTOR_FL Propeller {
      translation 0.15 0.15 0.025
      rotation 0 0 1 0
      shaftAxis 0 0 1
      centerOfThrust 0 0 0
      thrustConstants -12.5 0
      torqueConstants 0.0015 0
      fastHelixThreshold 50
      device RotationalMotor {
        name "motor_fl"
        maxVelocity 600
        maxTorque 30
      }
      fastHelix Solid {
        children [
          Shape {
            appearance PBRAppearance {
              baseColor 0.8 0.1 0.1
              transparency 0.5
            }
            geometry Cylinder {
              height 0.01
              radius 0.12
            }
          }
        ]
      }
      slowHelix Solid {
        children [
          Transform {
            rotation 1 0 0 1.5708
            children [
              Shape {
                appearance PBRAppearance {
                  baseColor 0.8 0.1 0.1
                }
                geometry Box {
                  size 0.24 0.01 0.02
                }
              }
            ]
          }
        ]
      }
    }
    
    DEF MOTOR_FR Propeller {
      translation 0.15 -0.15 0.025
      rotation 0 0 1 0
      shaftAxis 0 0 1
      centerOfThrust 0 0 0
      thrustConstants 12.5 0
      torqueConstants 0.0015 0
      fastHelixThreshold 50
      device RotationalMotor {
        name "motor_fr"
        maxVelocity 600
        maxTorque 30
      }
      fastHelix Solid {
        children [
          Shape {
            appearance PBRAppearance {
              baseColor 0.1 0.8 0.1
              transparency 0.5
            }
            geometry Cylinder {
              height 0.01
              radius 0.12
            }
          }
        ]
      }
      slowHelix Solid {
        children [
          Transform {
            rotation 1 0 0 1.5708
            children [
              Shape {
                appearance PBRAppearance {
                  baseColor 0.1 0.8 0.1
                }
                geometry Box {
                  size 0.24 0.01 0.02
                }
              }
            ]
          }
        ]
      }
    }
    
    DEF MOTOR_RL Propeller {
      translation -0.15 0.15 0.025
      rotation 0 0 1 0
      shaftAxis 0 0 1
      centerOfThrust 0 0 0
      thrustConstants 12.5 0
      torqueConstants 0.0015 0
      fastHelixThreshold 50
      device RotationalMotor {
        name "motor_rl"
        maxVelocity 600
        maxTorque 30
      }
      fastHelix Solid {
        children [
          Shape {
            appearance PBRAppearance {
              baseColor 0.1 0.1 0.8
              transparency 0.5
            }
            geometry Cylinder {
              height 0.01
              radius 0.12
            }
          }
        ]
      }
      slowHelix Solid {
        children [
          Transform {
            rotation 1 0 0 1.5708
            children [
              Shape {
                appearance PBRAppearance {
                  baseColor 0.1 0.1 0.8
                }
                geometry Box {
                  size 0.24 0.01 0.02
                }
              }
            ]
          }
        ]
      }
    }
    
    DEF MOTOR_RR Propeller {
      translation -0.15 -0.15 0.025
      rotation 0 0 1 0
      shaftAxis 0 0 1
      centerOfThrust 0 0 0
      thrustConstants -12.5 0
      torqueConstants 0.0015 0
      fastHelixThreshold 50
      device RotationalMotor {
        name "motor_rr"
        maxVelocity 600
        maxTorque 30
      }
      fastHelix Solid {
        children [
          Shape {
            appearance PBRAppearance {
              baseColor 0.8 0.8 0.1
              transparency 0.5
            }
            geometry Cylinder {
              height 0.01
              radius 0.12
            }
          }
        ]
      }
      slowHelix Solid {
        children [
          Transform {
            rotation 1 0 0 1.5708
            children [
              Shape {
                appearance PBRAppearance {
                  baseColor 0.8 0.8 0.1
                }
                geometry Box {
                  size 0.24 0.01 0.02
                }
              }
            ]
          }
        ]
      }
    }
    
    # Sensors
    InertialUnit {
      name "imu"
    }
    GPS {
      name "gps"
    }
    Gyro {
      name "gyro"
    }
    Compass {
      name "compass"
    }
    
    Camera {
      translation 0.06 0 0
      rotation 0 1 0 -0.2
      name "camera"
      width 640
      height 480
    }
  ]
  name "quadrotor"
  boundingObject Box {
    size 0.1 0.1 0.05
  }
  physics Physics {
    density -1
    mass 0.5
    centerOfMass [0 0 0]
  }
  controller "quadrotor_autopilot"
}
