import Simulator from "./rov/Simulator"
import RVOMath from "./rov/RVOMath"
import RovDebug from "./RovDebug";
import Vector2D from "./rov/Vector2D";

export default class RVO_test {
    private simulator: Simulator

    constructor() {

        console.log("================================================");
        console.log("RVO_test");
        console.log("================================================");
        this.simulator = new Simulator();
        let simulator = this.simulator;
        simulator.setTimeStep(0.5);
        simulator.setAgentDefaults(
            //在寻找周围邻居的搜索距离，这个值设置越大，会让小球在越远距离做出避障行为
            80, // neighbor distance (min = radius * radius)

            //寻找周围邻居的最大数目，这个值设置越大，最终计算的速度越 精确，但会加大计算量
            10, // max neighbors

            //计算动态的物体时的时间窗口
            100, // time horizon

            //代表计算静态的物体时的时间窗口，比如在RTS游戏中，小兵 向城墙移动时，没必要做出避障，这个值需要设置的很
            1, // time horizon obstacles

            //代表计算ORCA时的小球的半径，这个值不一定与小球实际显示的半径 一样，偏小有利于小球移动顺畅
            5, // agent radius

            //小球最大速度值
            2 , // max speed
            //初始速度
            // 0, // default velocity for x
            // 0, // default velocity for y
        )

        let counts = 1000;
        for (let i = 0; i < counts; i++) {
            // const angle = i * (2 * Math.PI) / counts;
            // let x = Math.cos(angle) * 200;
            // let y = Math.sin(angle) * 200;

            let x = 30 * Math.round(i / 5) - 50;
            if (i < counts >> 1) {
                x -= 100;
            } else {
                x += 100;
            }
            let y = 30 * (i % 10) - 50;
            // if (i < 50) y -= 200;

            simulator.addAgent(null);
            simulator.setAgentPosition(i, x, y);
        }

        for (let i = 0; i < simulator.getNumAgents(); i++) {
            let goal = simulator.getAgentPosition(i).clone()//.scale(-1);
            if (i < counts >> 1) {
                goal.x += 300;
            } else {
                goal.x -= 300;
            }
            // let goal = new Vector2D(100, 100);
            simulator.setAgentGoal(i, goal.x, goal.y);
        }



        let obstacle1: Vector2D[] = [];
        obstacle1.push(new Vector2D(-10.0, 40.0));
        obstacle1.push(new Vector2D(-80.0, 40.0));
        obstacle1.push(new Vector2D(-80.0, 10.0));
        obstacle1.push(new Vector2D(-10.0, 10.0));
        simulator.addObstacle(obstacle1);

        let obstacle2: Vector2D[] = [];
        obstacle2.push(new Vector2D(50.0, 60.0));
        obstacle2.push(new Vector2D(30.0, 30.0));
        obstacle2.push(new Vector2D(50.0, -10.0));
        obstacle2.push(new Vector2D(80.0, 20.0));
        obstacle2.push(new Vector2D(70.0, 60.0));
        simulator.addObstacle(obstacle2);

        // let obstacle3: Vector2D[] = [];
        // obstacle3.push(new Vector2D(100.0, -60.0));
        // obstacle3.push(new Vector2D(40.0, -60.0));
        // obstacle3.push(new Vector2D(40.0, -100.0));
        // obstacle3.push(new Vector2D(100.0, -100.0));
        // simulator.addObstacle(obstacle3);

        // let obstacle4: Vector2D[] = [];
        // obstacle4.push(new Vector2D(-100.0, -60.0));
        // obstacle4.push(new Vector2D(-100.0, -100.0));
        // obstacle4.push(new Vector2D(-40.0, -100.0));
        // obstacle4.push(new Vector2D(-40.0, -60.0));
        // simulator.addObstacle(obstacle4);

        simulator.processObstacles();

        // console.log(simulator)

        Laya.timer.frameLoop(1, this, this.step);
        RovDebug.ins.start(Laya.stage, simulator);
    }

    step() {
        let simulator = this.simulator;

        for (let i = 0; i < simulator.getNumAgents(); ++i) {
            if (RVOMath.absSq(simulator.getGoal(i).minus(simulator.getAgentPosition(i))) < RVOMath.RVO_EPSILON) {
                // Agent is within one radius of its goal, set preferred velocity to zero
                simulator.setAgentPrefVelocity(i, 0.0, 0.0);
                // console.log('finish ' + i);
            } else {
                // Agent is far away from its goal, set preferred velocity as unit vector towards agent's goal.

                // let v = RVOMath.normalize(simulator.getGoal(i).minus(simulator.getAgentPosition(i))).scale(simulator.agents[i].maxSpeed);
                let v = RVOMath.normalize(simulator.getGoal(i).minus(simulator.getAgentPosition(i))).scale(2);
                simulator.setAgentPrefVelocity(i, v.x, v.y);
            }
        }

        simulator.run();

        // console.log(simulator)
        if (simulator.reachedGoal()) {
            Laya.timer.clear(this, this.step);
            console.log('finish')
        }

    }

}