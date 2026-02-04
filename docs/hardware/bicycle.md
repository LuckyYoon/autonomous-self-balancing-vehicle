# Bicycle
![](https://retrospec.com/cdn/shop/products/Harper_2021_MatteBlack_1_1c9801fe-1b78-4fa8-985b-8b2d53d37c98.jpg?v=1769781311&width=1800)
## 1. Overview
The bicycle serves as the mechanical foundation for the **Autonomous Self-Balanced Bicycle** project. It provides the structural frame, wheelbase geometry, and mounting surfaces for all electrical, mechanical, and sensing subsystems.

The selected platform is the [**Harper Legacy Fixed Gear Bicycle**](https://retrospec.com/products/harper-fixie-bike-single-speed-legacy?variant=39428274356396&country=US&currency=USD&utm_medium=product_sync&utm_source=google&utm_content=sag_organic&utm_campaign=sag_organic&utm_source=google&utm_medium=pmax&utm_campaign=17394594692&nbt=nb%3Aadwords%3Ax%3A17394594692%3A%3A&nb_adtype=pla&nb_kwd=&nb_ti=&nb_mi=119671053&nb_pc=online&nb_pi=shopify_US_150506700830_39428274356396&nb_ppi=&nb_placement=&nb_li_ms=&nb_lp_ms=&nb_fii=&nb_ap=&nb_mt=&gad_source=1&gad_campaignid=17382889821&gbraid=0AAAAADFSHYfgdR-mhgBu6JFHLK7SBcKCm&gclid=CjwKCAiA0eTJBhBaEiwA-Pa-hWocUYpE2uWPqGhHPJ6PdHzGyQYhKpuNjQChuYBlPG1UiPKYFhL8NBoCafgQAvD_BwE) by Retrospec.


## 2. Bicycle Selection

### Selected Model
- **Model:** Harper Legacy Fixed Gear Bicycle  
- **Manufacturer:** Retrospec  
- **Drivetrain:** Fixed gear / single-speed  
- **Frame Material:** Steel (tubular construction)


## 3. Why This Bicycle Was Chosen

The Harper Legacy fixed gear bicycle was selected based on the following design considerations:

- **Tubular Frame Geometry**  
  The frame is composed almost entirely of round steel tubes, making it straightforward to design clamp-based or saddle-style mounts for sensors, batteries, and actuators.

- **Low Cost**  
  The bicycle is significantly cheaper than geared or suspension-equipped bicycles, reducing risk during early prototyping and modification.

- **Lightweight and Mechanically Simple**  
  As a fixed gear bicycle, it lacks derailleurs, shifters, suspension, and complex drivetrain components, minimizing weight and mechanical complexity.

- **Barebones Configuration**  
  The minimal stock configuration allows components to be added or replaced without needing to remove or work around unnecessary hardware.

- **Good Structural Rigidity**  
  The steel frame provides sufficient stiffness for mounting electronics while remaining tolerant of drilling, clamping, and repeated modifications.


## 4. Key Specifications

| Parameter              | Value / Description |
|------------------------|--------------------|
| Frame Material         | Steel |
| Frame Geometry         | Tubular, diamond frame |
| Drivetrain             | Fixed gear |
| Wheel Size             | 700 x 28c |
| Approx. Weight         | 11.33 kg |


## 5. Functional Role in the System
The bicycle provides:

- Structural support for all subsystems
- A known and repeatable mechanical geometry
- A real-world unstable platform for balance control
- A human-scale testbed for autonomous balancing and control algorithms

The bicycle itself is passive and does not provide active sensing or actuation beyond its mechanical dynamics.


## 6. Mechanical Integration Considerations

### 6.1 Mounting Strategy
- Tube-mounted clamps and use of braze on mounting points
- Non-permanent mounting where possible to allow iteration

### 6.2 Structural Load Paths
- Battery and electronics mass placed near the frame’s center and lower down
- Avoid excessive cantilevered loads from single tubes
- Reinforce mounts subjected to vibration or dynamic torque


## 7. Modifications from Stock Configuration

| Modification | Description |
|-------------|------------|
| Removal of unnecessary components | Decorative, non-functional components, and brakes removed |
| Added mounting interfaces | Custom clamps and brackets added |
| Cable routing | Re-routed or newly added cable paths |
| Weight redistribution | Electronics placed to minimize CG offset |


## 8. CAD & Mounting Interfaces

### Frame Reference Geometry
- All mounts reference **outer tube diameters**
- Tube diameters measured manually for CAD accuracy

### Mount Design Philosophy
- Clamp-based and use of Braze ons where possible
- No permanent frame modification unless required
- Designed for rapid removal and revision


## 9. Assembly Notes

1. Strip bicycle to bare functional components
2. Measure frame tube diameters
3. Install primary mounting brackets
4. Verify wheel alignment and steering freedom


## 10. Testing & Validation

### Static Validation
- Verify mounts do not slip under static load
- Check clearance at full steering lock
- Confirm no interference with drivetrain, motors, or reaction wheel

### Dynamic Validation
- Observe vibration and mount stability
- Inspect fasteners after initial runs


## 11. Failure Modes & Risks

| Risk | Cause | Mitigation |
|-----|------|-----------|
| Mount slippage | Insufficient clamp force | Increase contact area, add friction liner |
| Frame interference | Poor CAD assumptions | Physical measurement verification |
| Weight imbalance & balance failure | Poor mass placement | Reposition battery and electronics |


## 13. References
- [Retrospec Harper Legacy Owner's Manual](https://cdn.accentuate.io/150506700830/1720641000618/bike_ownersmanual.pdf?v=1720641000618)
- [Bicycle frame geometry reference guides](https://www.twowheeledwanderer.com/posts/bike-anatomy/)