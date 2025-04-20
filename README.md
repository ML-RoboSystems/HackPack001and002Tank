# IR Turret & Domino Robot Hack

This project is a **hardware _and_ software hack** that combines the CrunchLabs IR Turret and Domino Robot to create an interactive robotic tank. Make sure to get **both the hardware** and the **code** to make it work!

---

## 🔗 Links

- **IR Turret:** [IR Turret Product Page](https://www.crunchlabs.com/products/ir-turret)  
- **Domino Robot:** [Domino Robot Product Page](https://www.crunchlabs.com/products/domino-robot)  
- **Join the conversation on Discord:** [Discord Channel Link](https://discord.com/channels/1229106258749948056/1361423311241875459) *(Note: You must be part of the server to access this link)*

---

## 🛠️ Assembly Instructions

1. Remove the line sensors, servo, and limit switch (the black one with a metal bar) from the Domino Robot.
2. Remove all legs from the IR Turret except one. Take off the orange rubber pad from that leg.
3. Loosen the nut on the remaining leg, rotate it counter-clockwise by 90°, then retighten.
4. Place that leg inside the Domino Robot’s domino storage area.
5. Remove the acrylic gate (red transparent piece) and the black wood piece holding it. Screw the nuts back on. *(Leave the steel bars on.)*
6. Loosen the black rod (on top of the ball roller). Add 2 red washers to the long red screw, then tighten the rod back.
7. Remove the breadboard, Arduino, and power bank from the Domino Robot.
8. Use the red short screws to secure the orange nuts across from each other.  
   - When facing the robot from the front, the turret leg should be on the **right side**.
9. Remove the bottom left screw (front view), replace it with a red washer, and insert an orange screw through the IR Turret’s square hole, the washer, and the red rod underneath.
10. Remove the turret’s power bank holder (clear plastic strip) and place it between the breadboard and yaw servo. Use tape if needed.
11. Connect the **IR Turret Servo wires** to the **breadboard**:
    - Yellow → D12  
    - Green → D11  
    - Blue → D10  
    - Purple → D9
12. Insert the H-Bridge motor driver into the remaining breadboard space, making sure the four **3R3** blocks and **C106** face the back of the robot.
    - The H-Bridge (the red block) should be **right next** to the yellow, green, blue, and purple wires.
13. Upload the code (provided below), and you're good to go!

---

## 🎮 IR Remote Controls

| Button | Action              |
|--------|---------------------|
| 1      | Nothing             |
| 2      | Drive Forward       |
| 3      | Nothing             |
| 4      | Drive Left          |
| 5      | Nothing             |
| 6      | Drive Right         |
| 7      | Nothing             |
| 8      | Drive Backward      |
| 9      | Nothing             |
| 0      | Nothing             |
| *      | Nothing             |
| #      | Fire All Bullets    |
| ↑      | Barrel Up           |
| ↓      | Barrel Down         |
| ←      | Barrel Left         |
| →      | Barrel Right        |
| OK     | Fire One Bullet     |

---

## 📂 Code

[Link to the code](https://github.com/ML-RoboSystems/HackPack001and002Tank/blob/main/Tank...IR_Turret-Domino_Robot.ino)
