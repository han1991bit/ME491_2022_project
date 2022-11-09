PROJECT RULE
======

Project Overview
-----
![image1](image_1.png)
![image2](image_2.png)

Project Objectvie
-----
This project's goal is to train an agent that can **cross stepping stones and reach the goal**.
The goal is represented as a red sphere in the figure. 
You can refer RaiSim document website: [raisim_website](http://raisim.com/index.html). 
For your convenience, a basic template for the project is provided in the git repository. [git_repo](https://github.com/jhwangbo/ME491_2022_project). 
For evaluation, you need to **submit the video file** to represent your result.

Project Rule
-----
1. You can use any information given by the RaiSim engine.
2. You cannot modify the dynamics. Everyone should play in the same environment.
4. ### If you have any question, please upload issues on [git_repo](https://github.com/jhwangbo/ME491_2022_project). (We do not allow others)
5. The competition map will be provided 3 days before the submission date. You should have a general controller ready for the competition before that date. You will have 3 days to record a video for the submission.
6. You will also submit a runnable code. Your code should not depend on external libraries except the ones included.


Environment Condition
-----
1. There are total 5 flat surfaces (3m x 6m x 0.1m)
2. The horizontal gaps (x-directional) between the grounds are 20cm, 40cm, 60cm, 80cm each. In addition, there is random noise follows (5cm * N(0, 1)) that adds to the gap distance. 
3. The vertical distance (z-directional) of the surfaces : **1.0 + 0.01*N(0,1) m**  
4. The **last flat surface moves bi-directionally (x-directional) at a speed of 40cm/sec**. The direction might be changed 
every 1 second.
5. The goal position (red sphere) is identical with last one's center.

Submission
------
| Submission       | Date |
|------------------|------|
| Report, video, code submission  | before the competition (Dec 14th. 1pm) |


Competition
-----
We will watch the videos together on Dec 14th at 1pm.

Report
-----
Final report should contain these contents:
1. Problem definition (describe the MDP)
2. What algorithms did you use? How do they work (i.e., theory)?
3. Results. What contributed to the final performance the most?

The final report should not be more than 4 pages
