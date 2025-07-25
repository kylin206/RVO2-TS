import Simulator from '../Simulator';
import Vector2D from '../Vector2D';
import RVOMath from '../RVOMath';

describe('Simulator', () => {
    let simulator: Simulator;

    beforeEach(() => {
        simulator = new Simulator();
        simulator.setAgentDefaults(
            5,   // neighborDist
            10,  // maxNeighbors
            10,  // timeHorizon
            5,   // timeHorizonObst
            1,   // radius
            2,   // maxSpeed
            0, 0 // default velocity
        );
    });

    describe('initialization', () => {
        it('should initialize with default settings', () => {
            expect(simulator.getTimeStep()).toBe(0.25);
            expect(simulator.getNumAgents()).toBe(0);
            expect(simulator.getGlobalTime()).toBe(0);
        });

        it('should add agents correctly', () => {
            const agentId1 = simulator.addAgent(new Vector2D(0, 0));
            const agentId2 = simulator.addAgent(new Vector2D(2, 0));

            expect(simulator.getNumAgents()).toBe(2);
            expect(simulator.getAgentPosition(agentId1).x).toBe(0);
            expect(simulator.getAgentPosition(agentId2).x).toBe(2);
        });

        it('should add obstacles correctly', () => {
            const vertices = [
                new Vector2D(-1, -1),
                new Vector2D(1, -1),
                new Vector2D(1, 1),
                new Vector2D(-1, 1)
            ];

            const obstacleNo = simulator.addObstacle(vertices);
            simulator.processObstacles();

            const obstacles = simulator.getObstacles();
            expect(obstacles.length).toBe(4);
            expect(obstacles[0].next).toBe(obstacles[1]);
            expect(obstacles[3].next).toBe(obstacles[0]);
        });
    });

    describe('simulation loop', () => {
        it('should update agent positions correctly', () => {
            // 添加两个相向而行的agents
            const agent1 = simulator.addAgent(new Vector2D(-2, 0));
            const agent2 = simulator.addAgent(new Vector2D(2, 0));

            // 设置它们相向移动
            // let v1 = RVOMath.normalize(simulator.getGoal(agent1).minus(simulator.getAgentPosition(agent1))).scale(2);
            // let v2 = RVOMath.normalize(simulator.getGoal(agent2).minus(simulator.getAgentPosition(agent2))).scale(2);
            // simulator.setAgentPrefVelocity(agent1, v1.x, v1.y);   // 向右
            // simulator.setAgentPrefVelocity(agent2, v2.x, v2.y);  // 向左

            const originalPos1 = simulator.getAgentPosition(agent1).clone();
            const originalPos2 = simulator.getAgentPosition(agent2).clone();

            simulator.setAgentGoal(agent1, originalPos2.x, originalPos2.y);
            simulator.setAgentGoal(agent2, originalPos1.x, originalPos1.y);
            simulator.processObstacles();
            // 运行几个时间步
            for (let i = 0; i < 10; i++) {
                let v1 = RVOMath.normalize(simulator.getGoal(agent1).minus(simulator.getAgentPosition(agent1))).scale(2);
                let v2 = RVOMath.normalize(simulator.getGoal(agent2).minus(simulator.getAgentPosition(agent2))).scale(2);
                simulator.setAgentPrefVelocity(agent1, v1.x, v1.y);   // 向右
                simulator.setAgentPrefVelocity(agent2, v2.x, v2.y);  // 向左
                simulator.run();
            }

            // 检查agents是否移动了
            const newPos1 = simulator.getAgentPosition(agent1);
            const newPos2 = simulator.getAgentPosition(agent2);

            expect(newPos1.x).not.toBe(originalPos1.x);
            expect(newPos2.x).not.toBe(originalPos2.x);

            // 确保agents没有相撞（中心距离应该大于两者半径之和）
            const distance = RVOMath.abs(newPos1.minus(newPos2));
            expect(distance).toBeGreaterThan(simulator.getAgentRadius(agent1) + simulator.getAgentRadius(agent2));
        });

        it('should handle obstacle avoidance', () => {
            // 添加一个障碍物墙
            const wall = [
                new Vector2D(-3, 0),
                new Vector2D(3, 0),
                new Vector2D(3, 0.1),
                new Vector2D(-3, 0.1)
            ].reverse(); // 确保是逆时针顺序
            simulator.addObstacle(wall);
            simulator.processObstacles();

            // 添加一个想要穿过墙的agent
            const agent = simulator.addAgent(new Vector2D(0, -2));
            simulator.setAgentPrefVelocity(agent, 0, 1); // 向上移动

            // 运行模拟
            for (let i = 0; i < 10; i++) {
                simulator.run();
            }

            // 检查agent是否停在了墙前（y坐标应该小于墙的位置）
            const finalPos = simulator.getAgentPosition(agent);
            expect(finalPos.y).toBeLessThan(0);
        });

        it('should reach goals', () => {
            // 添加一个agent和它的目标
            const agent = simulator.addAgent(new Vector2D(0, 0));
            const goal = new Vector2D(4, 0);
            simulator.setAgentGoal(agent, goal.x, goal.y);

            // 设置移动方向
            simulator.setAgentPrefVelocity(agent, 1, 0);

            // 运行直到到达目标或超时
            let steps = 0;
            const maxSteps = 100;
            while (!simulator.reachedGoal() && steps < maxSteps) {
                simulator.run();
                steps++;
            }

            // 检查是否到达目标
            const finalPos = simulator.getAgentPosition(agent);
            const distToGoal = RVOMath.abs(finalPos.minus(goal));
            expect(distToGoal).toBeLessThan(0.1);
        });
    });

    describe('error handling', () => {
        it('should handle invalid agent indices', () => {
            expect(() => simulator.getAgentPosition(999)).toThrow();
        });

        it('should handle empty obstacle vertices', () => {
            expect(simulator.addObstacle([])).toBe(-1);
        });
    });
});