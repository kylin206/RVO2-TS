
import Vector2D from "./Vector2D";
import Obstacle from "./Obstacle";
import Agent from "./Agent";
import RVOMath from "./RVOMath";
import KdTree from "./KdTree";
import Line from "./Line";


export default class Simulator {

  agents: Agent[] = [];
  obstacles: Obstacle[] = [];
  private goals: Vector2D[] = [];
  kdTree: KdTree = new KdTree();

  timeStep = 0.25;

  private defaultAgent: Agent; // Agent

  private time = 0;

  constructor() {
    this.kdTree.simulator = this;
    this.kdTree.MAXLEAF_SIZE = 1000;
  }
  getGlobalTime(): number {
    return this.time;
  }

  getNumAgents(): number {
    return this.agents.length;
  }

  getTimeStep(): number {
    return this.timeStep;
  }

  setAgentPrefVelocity(i: number, vx, vy) {
    this.agents[i].prefVelocity = new Vector2D(vx, vy);
  }

  setAgentPosition(i: number, x: number, y: number) {
    this.agents[i].position = new Vector2D(x, y);
  }

  setAgentGoal(i: number, x: number, y: number) {
    this.goals[i] = new Vector2D(x, y);
  }

  setTimeStep(timeStep: number) {
    this.timeStep = timeStep;
  }

  getAgentPosition(i: number): Vector2D {
    return this.agents[i].position;
  }

  getAgentPrefVelocity(i: number): Vector2D {
    return this.agents[i].prefVelocity;
  }

  getAgentVelocity(i: number): Vector2D {
    return this.agents[i].velocity;
  }

  getAgentRadius(i: number): number {
    return this.agents[i].radius;
  }

  getAgentOrcaLines(i: number): Line[] {
    return this.agents[i].orcaLines;
  }

  addAgent(position: Vector2D = null) {
    if (!this.defaultAgent) {
      throw new Error("no default agent");
    }

    if (!position) position = new Vector2D(0, 0);

    var agent = new Agent();

    agent.position = position;
    agent.maxNeighbors = this.defaultAgent.maxNeighbors;

    agent.radius = this.defaultAgent.radius;
    agent.maxSpeed = this.defaultAgent.maxSpeed;
    agent.neighborDist = this.defaultAgent.neighborDist;
    agent.timeHorizon = this.defaultAgent.timeHorizon;
    agent.timeHorizonObst = this.defaultAgent.timeHorizonObst;
    agent.velocity = this.defaultAgent.velocity;
    agent.simulator = this;

    agent.id = this.agents.length;
    this.agents.push(agent);
    this.goals.push(position);

    return this.agents.length - 1;
  }

  //  /** float */ neighborDist, /** int */ maxNeighbors, /** float */ timeHorizon, /** float */ timeHorizonObst, /** float */ radius, /** float*/ maxSpeed, /** Vector2 */ velocity)
  setAgentDefaults(neighborDist: number,
    maxNeighbors: number,
    timeHorizon: number,
    timeHorizonObst: number,
    radius: number,
    maxSpeed: number,
    velocityX: number = 0, velocityY: number = 0) {
    if (!this.defaultAgent) {
      this.defaultAgent = new Agent();
    }

    this.defaultAgent.maxNeighbors = maxNeighbors;
    this.defaultAgent.maxSpeed = maxSpeed;
    this.defaultAgent.neighborDist = neighborDist;
    this.defaultAgent.radius = radius;
    this.defaultAgent.timeHorizon = timeHorizon;
    this.defaultAgent.timeHorizonObst = timeHorizonObst;
    this.defaultAgent.velocity = new Vector2D(velocityX, velocityY);
    this.defaultAgent.simulator = this;
  }

  run() {
    this.kdTree.buildAgentTree();

    for (var i = 0; i < this.getNumAgents(); i++) {
      this.agents[i].computeNeighbors();
      this.agents[i].computeNewVelocity();
      this.agents[i].update();
    }

    this.time += this.timeStep;
  }

  reachedGoal(): boolean {
    let pos: Vector2D;
    for (var i = 0, len = this.getNumAgents(); i < len; ++i) {
      pos = this.getAgentPosition(i);
      if (RVOMath.absSq(this.goals[i].minus(pos)) > RVOMath.RVO_EPSILON) {
        return false;
      }
    }
    return true;
  }

  addGoals(goals: Vector2D[]) {
    this.goals = goals;
  }

  getGoal(goalNo: number): Vector2D {
    return this.goals[goalNo];
  }

  addObstacle(vertices: Vector2D[]): number {
    if (vertices.length < 2) {
      return -1;
    }

    var obstacleNo = this.obstacles.length;

    for (var i = 0, len = vertices.length; i < len; ++i) {
      var obstacle = new Obstacle();
      obstacle.point = vertices[i];
      if (i != 0) {
        obstacle.previous = this.obstacles[this.obstacles.length - 1];
        obstacle.previous.next = obstacle;
      }
      if (i == vertices.length - 1) {
        obstacle.next = this.obstacles[obstacleNo];
        obstacle.next.previous = obstacle;
      }
      obstacle.unitDir = RVOMath.normalize(vertices[(i == vertices.length - 1 ? 0 : i + 1)].minus(vertices[i]))

      if (vertices.length == 2) {
        obstacle.isConvex = true;
      } else {
        obstacle.isConvex = (
          RVOMath.leftOf(vertices[(i == 0 ? vertices.length - 1 : i - 1)],
            vertices[i], vertices[(i == vertices.length - 1 ? 0 : i + 1)]) >= 0);
      }

      obstacle.id = this.obstacles.length;

      this.obstacles.push(obstacle);
    }

    return obstacleNo;
  }

  processObstacles() {
    this.kdTree.buildObstacleTree();
  }

  queryVisibility(point1: Vector2D, point2: Vector2D, radius: number): boolean {
    return this.kdTree.queryVisibility(point1, point2, radius);
  }

  getObstacles(): Obstacle[] {
    return this.obstacles;
  }

}
