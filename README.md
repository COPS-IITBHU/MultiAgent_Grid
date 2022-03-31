# Multi-Agent Reinforcement Leaning


We use Multi Agent Deep Deterministic Policy Gradient to train agents on our custom environment.
<!---
<p align="center">
<img src="./img/1.png"  />
</p>

<p align="center">
<img src="./img/2.png"  />
</p>
-->

|<img src="./img/1.png" />|<img src="./img/2.png" width ="50%"  />|
| -------------- | --------------- |
| Algorithm      |  Environemnt    |

In our setup, we take action as the control points for planning our trajectory using bezier curves.The trajectory followed by the agent is taken as observation.
<br>
The reward structure is as follows:
<br>
<ul>
 <li>-10 for any collision
 <li>+1 for reaching the target.
 </ul>



## Setup
<hr>

``` console
git clone 'link'
cd 'repo_name'
pip install -e .
```
## Training
<hr>


``` console
python train.py
```

## Evaluation
<hr>

```console
python eval.py
```
![Alt Text](./img/3.gif)


