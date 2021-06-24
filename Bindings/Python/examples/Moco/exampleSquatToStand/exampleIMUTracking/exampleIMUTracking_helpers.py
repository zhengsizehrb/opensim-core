from opensim import *
import numpy as np
import matplotlib.pyplot as plt

def addCoordinateActuator(model, coordName, optForce):
    coordSet = model.updCoordinateSet()
    actu = CoordinateActuator()
    actu.setName('tau_' + coordName)
    actu.setCoordinate(coordSet.get(coordName))
    actu.setOptimalForce(optForce)
    actu.setMinControl(-1)
    actu.setMaxControl(1)

    # Add to ForceSet
    model.addForce(actu)


def getTorqueDrivenSquatToStandModel():

    # Load the base model.
    model = Model('../squatToStand_3dof9musc.osim')

    # Remove the muscles in the model.
    model.updForceSet().clearAndDestroy()
    model.initSystem()

    # Add CoordinateActuators to the model degrees-of-freedom.
    addCoordinateActuator(model, 'hip_flexion_r', 150)
    addCoordinateActuator(model, 'knee_angle_r', 300)
    addCoordinateActuator(model, 'ankle_angle_r', 150)

    return model


def addIMUFrame(model, bodyName, translation, orientation):
    body = model.updBodySet().get(bodyName)
    name = str(body.getName()) + '_imu_offset'
    bodyOffset = PhysicalOffsetFrame(name, body, Transform())
    bodyOffset.set_translation(translation)
    bodyOffset.set_orientation(orientation)
    body.addComponent(bodyOffset)
    model.finalizeConnections()


def plotAccelerationSignals(*args):
    args_list = list(args)

    accelerationsReference = args_list[0]
    if len(args) == 2:
        accelerationsTracking = args_list[1]

    # Create a time vector that can be used for plotting
    timeVec = accelerationsReference.getIndependentColumn()
    time = np.zeros(len(timeVec))
    for i in np.arange(len(timeVec)):
       time[i] = timeVec[i]
    
    fig = plt.figure()
    # Plot the torso accelerations
    ax = fig.add_subplot(131)
    torso = accelerationsReference.getDependentColumn(
            '/bodyset/torso/torso_imu_offset')
    ax.plot(time, torso[:,1], color='red', linewidth=3,
        label='predict')
    if len(args) == 2:
        torsoTrack = accelerationsTracking.getDependentColumn(
            '/bodyset/torso/torso_imu_offset')
        ax.plot(time, torsoTrack[:,1], color='blue', 
            linestyle='--', linewidth=3, label='track')

    if len(args) == 2:
        ax.legend(loc='best')

    ax.set_title('torso')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('acceleration (m/s^2)')

    # Plot the femur accelerations
    ax = fig.add_subplot(132)
    femur = accelerationsReference.getDependentColumn(
        '/bodyset/femur_r/femur_r_imu_offset')
    ax.plot(time, femur[:,1], color='red', linewidth=3)
    if len(args) == 2:
        femurTrack = accelerationsTracking.getDependentColumn(
            '/bodyset/femur_r/femur_r_imu_offset')
        ax.plot(time, femurTrack[:,1], color='blue', 
            linestyle='--', linewidth=3)

    ax.set_title('femur')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('acceleration (m/s^2)')

    # Plot the tibia accelerations
    ax = fig.add_subplot(133)
    tibia = accelerationsReference.getDependentColumn(
        '/bodyset/tibia_r/tibia_r_imu_offset')
    ax.plot(time, tibia[:,1], color='red', linewidth=3)
    if len(args) == 2:
        tibiaTrack = accelerationsTracking.getDependentColumn(
            '/bodyset/tibia_r/tibia_r_imu_offset')
        ax.plot(time, tibiaTrack[:,1], color='blue', 
            linestyle='--', linewidth=3)
    
    ax.set_title('tibia')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('acceleration (m/s^2)')
