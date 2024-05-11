# Hip Cover & Cable Harness Sub-Assembly

[TOC]

## Overview

The hip cover is a complex scaffolding and wiring structure that tunnels power
and signal from the torso of the robot into the trimotor assemblies and back.

NOTE: In the partially-assembled robot below, we have a Front Right leg (*screen
left*) with the hip cover installed compared to a leg sans cover (*right side*).
The entire hip cover and cable harness sub-assembly is highlighted purple.
![Side-by-side Example of Leg with Hip Cover Installed vs Barren Leg](images/robot_assembly/hip_cover/hip_cover_installed_highlighted_and_without.jpg)

This is the path upon which ECAT signal travels down. It starts from the torso,
and next proceeds into the abduction, then into the hip, and finally terminating
at the knee before returning to the torso.

NOTE: An inside and outside view of a hip cover and its cable harness assembly.
*Parts have been highlighted to group them with regards to direction &
orientations.*
![Hip Cover Inside and Outside View](images/robot_assembly/hip_cover/hip_motor_in_out_highlighted_transparent.png)
Everything highlighted **BLUE** resides inside the hip. \
**RED** is used to denote the external hip features. \
The **GREEN** bundle travels through a center hole in the cover into the knee. \
And the **YELLOW** group funnels through the topside hole into the torso.

And it comes in FOUR DISTINCT BUILD CONFIGURATIONS:

1.  **FL** \(**F**ront **L**eft\)
2.  **HL** \(**H**ind **L**eft\)
3.  **FR** \(**F**ront **R**ight\)
4.  **HR** \(**H**ind **R**ight\)

This dictates the specific lengths of cables needed to cross over the physical
space inside the robot in order to reach power and communication.

## Parts Needed

Separating the inventory chart into three distinct "chunks" so we can better
parse how the disparate power and signal cables come together based on the
specific assembly configuration we are making.

### EtherCAT Cables

Five cables are involved in I/O for the EtherCAT signal.

| Part Name          | Quantity | Description & Details                        |
| ------------------ | :------: | -------------------------------------------- |
| A2H ECAT           | 1        | Transfers data from **A**bduction to (**2**) |
:                    :          : **H**ip Gebru; Cable 15                      :
| H2K ECAT           | 1        | Communication between **H**ip to (**2**)     |
:                    :          : **K**nee PCBAs; Cable 16                     :
| K2H Cover ECAT     | 1        | One half of the "return cable" from **K**nee |
:                    :          : through the **H**ip *and back into the       :
:                    :          : torso*; Cable 17                             :
| Hip Return SS ECAT | 1/0      | Second half of the "return cable"; used if   |
:                    :          : the following leg is on the **S**ame         :
:                    :          : **S**ide; Cable 18                           :
| Hip Return OS ECAT | 0/1      | Alternative complement for "return cable" if |
:                    :          : the next joint is on the **O**pposite        :
:                    :          : **S**ide; Cable 19                           :

WARNING: Front Left (FL) and Rear Left (RL) have special incoming and outgoing
cables dependent on the (DFB) that are NOT LISTED HERE (see cable drawings). The
reason being that FL and RL are the first and last limb in the ECAT chain,
respectively.

### Power Distribution Cables

Three distinct cables are used for power distribution.

| Part Name         | Quantity | Description & Details                         |
| ----------------- | :------: | --------------------------------------------- |
| Hip Power Distro  | 1        | Cable 4; plugs directly into the Fei-Fei PCBA |
:                   :          : so **all build configurations will use ONE of :
:                   :          : these**                                       :
| Fei-Fei to A+H FR | 1/0      | Distributes power to **A**bduction and        |
:                   :          : **H**ip; exclusively used for **FR**ont legs; :
:                   :          : Cable 2                                       :
| Fei-Fei to A+H RE | 0/1      | Only for **RE**ar cable assemblies; Cable 3   |

![Three Unique Power Cables for the Hip Cover](images/robot_assembly/hip_cover/three_power_cables.jpg)

WARNING: Do not repair faulty cables. If a pin is loose or sleeving is damaged
or connectivity is suspect, cut and **throw away** the cable.

### Mechanical Parts

The mechanical components will be the same regardless of the previous cable
combinations.

| Part Name              | Quantity    | Description & Details                 |
| ---------------------- | :---------: | ------------------------------------- |
| Hip Cover              | 1           | 3D printed part that shields the hip  |
:                        :             : PCBA;                                 :
| Cable Clamps           | 2           | Complementary archpieces to the hip   |
:                        :             : cover that serve as cable retention   :
| M3x12mm FHCS           | 4           | Countersunk bolt that secures the     |
:                        :             : cable clamps into the hip-cover heat  :
:                        :             : inserts                               :
| Braided Cable Wrapping | *as needed* | For organizing the cable bundles; we  |
:                        :             : cut two pieces per harness\: 180mm x1 :
:                        :             : and 250mm x1                          :
| Torso Grommet          | 1           | Paired 3D-printed parts that retain   |
:                        :             : the bundle to torso                   :
| M3x10mm SHCS           | 2           | Secures torso grommet mated pieces    |
| Knee Cable Entry Gland | 1           | Two mated 3D-printed pieces that      |
:                        :             : retain the cable running into the     :
:                        :             : knee                                  :
| M3x18 FHCS BO          | 2           | Counter-sunk bolts that keeps the     |
:                        :             : knee gland parts together             :

## Procedure

1.  Combine the appropriate power cable pair and affix it to the left side of
    the internal Hip Cover.

    A. Staring with various empty Hip Covers. *In this example we've already
    specified limb designation in advance for the entire batch based on build
    need.*

    ![Hip Covers Only with Designations](images/robot_assembly/hip_cover/hip_covers_only.jpg)

    NOTE: We have a cover meant for a **Front Left** leg so the following
    procedure will be based on that particular build configuration. *The
    necessary heat inserts should already be installed.*
    ![Hip Cover FL](images/robot_assembly/hip_cover/hip_cover_fl.jpg)

    B. It will receive the corresponding pair: x1 `Hip Power Distro` and x1
    `Fei-Fe to A+H FR`
    ![Front Left Fei Fei Cable Pair](images/robot_assembly/hip_cover/FL_feifei_pairing.jpg)

    C. Secure the junction of the power cable pair with a zip-tie around the
    `XT60H-M`(ale) connector as shown. Slowly tighten with a zip-tie gun to
    approximate tightness setting: 2.5
    ![FL Power Cable Zip tied](images/robot_assembly/hip_cover/feifei_pair_zip_tied.jpg)
    ![Close-up FL Power Cable Pair](images/robot_assembly/hip_cover/feifei_pair_zip_tied_closeup.jpg)

2.  Assemble the corresponding ECAT "return cable" duo and affix it to the
    right-side slot of the internal Hip Cover.

    NOTE: Mark all ends with their corresponding **I**nput or **O**utput
    (*arbitrary designation*) so they don't get mixed up or plugged in the wrong
    direction once finalized.
    ![Input and Output](images/robot_assembly/hip_cover/ECAT_marked_IO.jpg)

    A. The current limb is Front Left (FL). *Therefore the following joint must
    be Front Right (FR)*, which is on the SAME SIDE. **Thus we use the required
    x1 `K2H Cover ECAT` along with a x1 `Hip Return SS ECAT`**

    NOTE: The "return cable" with a mark to denote the junction between a `K2H`
    cable and either a `SS` or `OS` cable, depending on what specific leg comes
    next.
    ![Paired Return Cable](images/robot_assembly/hip_cover/paired_return_ECAT_cable.jpg)

    B. Secure the junction on the remaining right-side slot with a zip-tie
    across the `Molex 2014441110` as shown and tighten to spec like before.
    ![Return ECAT Zip Tied](images/robot_assembly/hip_cover/return_cable_zipped.jpg)

3.  Slot in the Abduction (output side) to Hip (input side) Ethercat cable.

    A. `A2H ECAT` goes in through he topside hole from the outside.
    ![Abduction into Hip Input through Hip Cover](images/robot_assembly/hip_cover/abduction_2_hip_input.jpg)

    B. We zip-tied it on the external side to preserve bundle length.
    ![A2H Minimum Length Zip Tied (internal view)](images/robot_assembly/hip_cover/A2H_minimum_internal_length.jpg)

    *It will need ~3 inches into the motor in order to properly reach the hip
    Gebru*.
    ![A2H Minimum Length Zip Tied (External view) with partial view of H2K](images/robot_assembly/hip_cover/A2H_minimum_length_external_view.jpg)

4.  Install the Hip (output end) to Knee (input) Ethercat cable. A. The `H2K
    ECAT` cable will tunnel from inside the hip cover and through the middle
    opening to the outside.

    ![H2K Minimum Length Zip Tied](images/robot_assembly/hip_cover/H2K_minimum_length_zip_tied.jpg)

    B. *This goes further downstream into the KNEE joint, which is last in the
    trimotor chain.*

5.  Wrap and secure the external cable bundles.

    A. Cut a pair of braided cable sleeving to lengths of 180mm and 250mm.
    ![Cable Sleeve 180mm](images/robot_assembly/hip_cover/cable_sleeve_180mm.jpg)
    ![Cable Sleeve 250mm](images/robot_assembly/hip_cover/cable_sleeve_250mm.jpg)

    B. Wrap the external cable bundles as shown.

    NOTE: The shorter 180mm length will exit to topside hole and run towards the
    torso. The longer 250mm variant covers the cables coming out from the center
    hole and into the knee.
    ![Cable Bundles, Lengths, and Directions](images/robot_assembly/hip_cover/bundling_cables.jpg)

    C. Ensure the ends of the bundles are slightly crossing through the openings
    so the clamps will secure them in the following step.

    D. Each hole exiting the hip cover has a complementary clamp piece that is
    secured with x2 m3x12mm bolts. Tighten until the cable feels anchored.
    ![Secured Cable Clamps](images/robot_assembly/hip_cover/secure_cable_clamps.jpg)

NOTE: For the following steps we'll be applying the additional scaffolding for
the cable bundle outputs. In the below example we have a torso grommet pair
(*left*) and the knee gland assembly. (right).
![Grommet and Wallace Parts](images/robot_assembly/hip_cover/torso_grommet_and_knee_gland.jpg)
*Prior to the installation we cut out two aluminum rods to 120mm and 190mm in
order to serve as "rulers" to ensure enough slack for cables to reach their
intended destinations.*
![Aluminum 120mm Guide Into Torso](images/robot_assembly/hip_cover/guide_to_torso_120mm.jpg)
![Aluminum 190mm Guide Into Knee](images/robot_assembly/hip_cover/guide_to_knee_190mm.jpg)

1.  Install the Torso Grommet {value=6}

    A. Confirm distance before marking and clamping. Make sure the cables aren't
    all tangled up inside the bungle, which could lead to pinched wires.
    ![Measuring for Torso Grommet](images/robot_assembly/hip_cover/measuring_for_grommet.jpg)

    B. Also consider the **ORIENTATION** of the mated halves, as they need to be
    installed as shown to easily slot into the torso with minimal cable strain.
    ![Torso Grommet Orientation](images/robot_assembly/hip_cover/measuring_for_grommet_orientation.jpg)

    C. Clamp the halves together, aligning openings to heat inserts, and
    screwing down x2 M3x10mm bolts to hand-tightness.
    ![Grommet Secured Close-up](images/robot_assembly/hip_cover/grommet_secured.jpg)

2.  Apply Knee Entry Gland

    A. Measure distance and access cable "health" as before.
    ![Measuring for Knee Gland](images/robot_assembly/hip_cover/measuring_for_knee_gland.jpg)

    B. Confirm orientation of the knee gland pieces with respect to the cable
    bundle, as shown:
    ![Knee & Cable Orientation](images/robot_assembly/hip_cover/knee_gland_orientation.jpg)

    WARNING: The knee gland pieces can be connected incorrectly. This creates a
    much tighter opening which can damage the cable bundle going into the knee
    *on top of* not properly fitting into the joint it's traveling towards.
    ![Knee Gland Pairs Wrong vs Correct](images/robot_assembly/hip_cover/knee_gland_orientation_isolation.jpg)

    C. Clamp down the knee gland pairs and fasten the x2 M3x18mm bolts to
    hand-tightness.

NOTE: Example of a completed **F**ront **L**eft (**FL**) hip cover & cable
harness sub-assembly. REMEMBER: there are minor differences with regards to
cable lengths, combinations, and port orientation depending on which specific
leg you are building for.
![Completed Front Left Hip Cover & Cable Harness](images/robot_assembly/hip_cover/front_left_hip_cover_cable_harness_complete.jpg)
