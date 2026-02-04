# Drive Motor


![](https://monsterscooterparts.com/cdn/shop/files/c80-8731_4_4.jpg?v=1712701265)


## 1. Overview
The drive motor provides propulsion for the Autonomous Self-Balanced Bicycle. It delivers controlled torque to the rear wheel through a chain drive, enabling low-speed motion for autonomous operation.

The selected motor is the **MY1016 24V 250W DC motor**.


## 2. Motor Selection

### Selected Model
- **Model:** MY1016  
- **Motor Type:** Brushed DC  
- **Rated Voltage:** 24 V  
- **Rated Power:** 250 W  
- **Output Drive:** 11T 25H chain sprocket


## 3. Why This Motor Was Chosen

The MY1016 motor was selected based on practical and mechanical integration considerations:

- **Availability**  
  The motor was already available on hand, which made it an easy candidate for the bicycle.

- **Standard 25H Chain Sprocket**  
  The motor includes a pre-installed 11T 25H sprocket, making it easy to source a compatible rear sprocket for the bicycle's standard 1.375" x 24 TPI threads without requiring a custom adapter.

- **Compact Form Factor**  
  The motor is relatively small, allowing it to be mounted within the bicycle frame without excessive interference.

- **Threaded Mounting Holes**  
  The base of the motor includes threaded mounting holes, simplifying mechanical mounting and enabling a rigid, bolt-on motor mount design.

- **Adequate Power for Low-Speed Operation**  
  The 250 W rating is sufficient for controlled low-speed motion.


## 4. Key Specifications

| Parameter            | Value |
|---------------------|-------|
| Motor Type          | Brushed DC |
| Rated Voltage       | 24 V |
| Rated Power         | 250 W |
| Rated Speed         | ~2650 RPM |
| Output Sprocket     | 25H |
| Mounting Interface  | Threaded base holes |
| Cooling             | Passive air cooling |


## 5. Functional Role in the System
The drive motor:

- Drives the rear wheel through a chain transmission
- Enables forward and reverse motion
- Allows fine torque control through the ESC for low-speed maneuvers

The motor does not contain internal sensing and relies on external control and feedback systems.


## 6. Electrical Integration

### 6.1 Power
- **Supply Voltage:** 24 V battery system
- **Peak Current:** Determined by motor controller and load
- **Protection:** Fuse and controller current limiting

### 6.2 Signals
The motor is driven by a motor controller and does not directly interface with logic-level signals.

| Connection | Type | Connected To | Notes |
|-----------|------|--------------|-------|
| Motor +   | Power | Motor Controller | Polarity controlled |
| Motor −   | Power | Motor Controller | Reversible |


## 7. Mechanical Integration

### 7.1 Mounting
- Motor mounted via base plate using threaded holes
- Custom 3D-printed mount attaches motor to bicycle seat tube
- Mount designed to resist chain tension forces

### 7.2 Drivetrain Interface
- 25H chain connects motor sprocket to rear wheel sprocket
- Chain alignment critical to reduce wear and noise
- Chain tension is primarily adjusted by sliding the rear wheel within the frame dropouts, followed by an axle-mounted tensioner.


## 8. CAD & Mount Design

### [Drive Motor Mount](../cad/drive_motor_mount.stl)

### Mounting Interface
- Threaded holes on motor base used as primary attachment points
- Used clamps around seat tube and braze on holes for secure mounting to frame
- CAD model references measured motor dimensions

### Design Considerations
- Maintain chain alignment
- Minimize added mass and overhang
- Provide resistance against motor torque and chain tension


## 9. Assembly Notes

1. Attach motor to mount using base mounting holes
2. Secure mount to bicycle frame
3. Install chain and adjust tension
4. Verify sprocket alignment
5. Confirm free rotation without binding


## 10. Testing & Validation

### Bench Testing
- Verified motor rotation at nominal voltage with bicycle upside down
- Checked current draw under no-load conditions

### On-Bike Testing
- Low-speed drive test
- Observed vibration and mount stability
- Verified chain alignment and tension


## 11. Failure Modes & Risks

| Failure Mode | Cause | Mitigation |
|-------------|------|-----------|
| Chain derailment | Poor alignment | Precise mount positioning & chain tensioner |
| Overheating | Sustained high load | PWM limits |
| Mount loosening | Vibration | Threadlocker, lock washers |


## 12. Lessons Learned

- Compatible sprocket was a big help in designing and building the system
- Threaded motor bases significantly increase mounted strength and reduce vibrations


## 13. References
- [MY1016 Motor Technical Drawing](https://cart.electricscooterparts.com/resize/Shared/images/product/24-Volt-250-Watt-2650RPM-Electric-Scooter-Motor/MOT-24250-DIMENSIONS.png?bw=1000&w=1000&bh=1000&h=1000)