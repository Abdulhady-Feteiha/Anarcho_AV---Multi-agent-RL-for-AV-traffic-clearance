# Anarcho_AV---Multi-agent-RL-for-AV-traffic-clearance

This repo integrates a DQN with autonomous driving in SUMO environment. The problem is to find a set of actions for all vehicles in the simulation such that the cooperative behaviour of the group results in road evacuation for an emergency vehicle. <br> 
The work was published in the RTATM2021 conference titled: "Coordinated traffic clearance for emergency vehicles using rein- forcement learning" <br>

Under supervision of **Dr. Mohamed El-Shenawy** <br>
melshenawy@zewailcity.edu.eg <br> <br>

For more information on the RL-Sumo model, feel free to contact: <br>
* **Alaa Hesham**: s-alaahesham@zewailcity.edu.eg
* **Waleed Awad**: s-wmustafa@zewailcity.edu.eg
* **Abdulhady Feteiha**: s-abdulhady@zewailcity.edu.eg <br>

A parallel effort is exerted to construct a ROS (Robot operating system) model along with hardware equipped autonomous vehicles. For more information on that part, please refer to our Robotics heroes: <br>
* **Tasneem Omara**: s-tsneem.omara@zewailcity.edu.eg
* **Mohamed El-Sayed**: s-mohammedelsayed@zewailcity.edu.eg
* **Nadine Amr**: s-nadine.amr@zewailcity.edu.eg 

# Requirements <br>
* The sumo version used is a nightly snapshot of past March 19, 2020. Please refer to sumo for Installation and debugging: https://sumo.dlr.de/docs/Downloads.html#nightly_snapshots <br>
* to install the required libraries: <br>
  ` pip install -r ./requirements.txt ` <br>
# Train and Save a Q table <br>
`cd Anarcho1.3` <br>
`python Anarcho.py --Train` <br>
# Test a saved Q table <br>
`cd Anarcho1.3` <br>
`python Anarcho.py --Test` <br>


# Versions Contents:
* **Anarcho1.3**: Single RL Agent  + One Ambulance Car 
* **Anarcho1.4**: Mutiple Agents (RL/Auto) + One Ambulance Car, with control over lane busyness, rl_percentage ..etc. Demonstration only, no learning. 
* **Anarcho1.41**: Multiple RL Cars, same as 1.4, but should have independent learners. Still not complete.
* **Anarscho1.5**: Enables running experiments. Introduces the concept of sample(dictionary) and experiment to produce outputs. Still not complete. Cloned from Anarcho1.41.

-------------------------------------------------------------------------- <br>
![ZC](ZC.png)
