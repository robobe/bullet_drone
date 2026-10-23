# Quadcopter simulation

Shared language for describing the simulated vehicle, its motion, and its observation.

## Language

**Quadcopter**:
A vehicle with four rotors whose combined thrust and torque determine its motion.

**Rotor command**:
A requested angular speed for each of the quadcopter's four rotors, in a declared rotor order.

**Wrench command**:
A requested collective thrust and body torque to be produced by the rotors.

**Motor allocation**:
The distribution of a wrench command across rotor outputs, together with the achievable thrust and torque after actuator limits.

**Ideal hover**:
An equilibrium from level rest where rotor thrust balances weight and combined torque is zero; it does not imply active stabilization.

**World frame**:
The fixed right-handed reference frame of the scene, with X/Y horizontal and Z upward.

**Body frame**:
The drone-attached right-handed reference frame, with X forward, Y left and Z upward.

**Frame transformation**:
A change in the coordinates used to express a physical vector or orientation; it does not change the physical quantity.

**Drone state**:
The quadcopter's physical condition at a specific simulation time, including its pose, velocities, and rotor speeds.

**Ground truth**:
The authoritative simulated physical condition before measurement noise or sensor limitations are applied.

**Sensor reading**:
A simulated measurement produced by a sensor at its sampling time, including its validity and measurement uncertainty effects.

**Drone observation**:
A ground-truth drone state paired with the latest readings from the drone's sensors, whose individual sampling times may differ.

**IMU**:
An inertial measurement unit that measures specific force and angular velocity in its sensor frame; it does not directly measure position or orientation.

**Barometer**:
A sensor that measures atmospheric pressure, from which altitude relative to a declared reference can be derived.

**Rangefinder**:
A sensor that measures distance to the first detectable surface along its sensing direction, within its operating range.

**Simulation time**:
Time elapsed within a simulation episode, independent of elapsed wall time.

**Episode**:
A continuous simulation run between initialization or reset and the next reset or termination.

**Scene snapshot**:
The poses of the drone and other visible objects at one simulation time, together with the identity of the scene they belong to.

**Attached camera**:
A camera whose pose is fixed relative to the drone, including a camera mounted beneath it.

**Tracking camera**:
A camera that observes the drone from a separately defined viewpoint and follows its motion.

**Image frame**:
A camera observation associated with a specific scene snapshot and camera configuration.

**Telemetry sample**:
A time-stamped observation of drone state and associated commands or measurements for analysis.

**Air-relative velocity**:
The vehicle or rotor-point velocity relative to the local moving air, expressed in a declared frame.

**Ambient wind**:
Air velocity supplied by the environment independently of the drone's rotor wakes.

**Propwash**:
The downstream airflow produced by a rotor, which changes local air-relative velocity where the wake reaches other parts of the drone.

**Ground effect**:
A rotor thrust increase associated with a nearby supporting surface constraining its downwash.
