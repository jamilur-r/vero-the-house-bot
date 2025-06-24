# 🤖 Vero The House Bot 🚀

Welcome to **Vero**, your friendly neighborhood house bot!  
She moves, she grooves, she... sometimes bumps into things.  
Let's get this party started! 🎉

---

## 🏁 Starting Up

### 1. Summon the Bot (aka: Run the Listener)

Open your terminal and cast this ancient spell:

```bash
ros2 run vero_core move_node
```
*Vero awakens... (insert dramatic robot noises here)*

---

### 2. Command the Bot (aka: Send Movement Instructions)

Want Vero to stop, go, or do the robot dance?  
Send her a message like this:

```bash
ros2 topic pub /move_cmd std_msgs/String "data: 'stop'"
```
*Pro tip: Replace `'stop'` with `'forward'`, `'backward'`, `'left'`, `'right'`, or `'moonwalk'` (okay, maybe not moonwalk... yet).*

---

## 🐸 Bonus Meme

> "Why did the robot cross the road?  
> To get to the ROS-side!"

---

Happy botting! If Vero starts plotting world domination, just unplug her.

### Useful commands 

for creating a new package:
```bash
cd src
ros2 pkg create --build-type ament_python <package_name>
```
