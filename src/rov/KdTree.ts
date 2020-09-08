
import RVOMath from "./RVOMath";
import Simulator from "./Simulator";
import Agent from "./Agent";
import Obstacle from "./Obstacle";
import Vector2D from "./Vector2D";

export default class KdTree {
  simulator: Simulator;

  MAXLEAF_SIZE = 100;

  private agents: Agent[] = [];
  private agentTree: AgentTreeNode[] = [];
  private obstacleTree: ObstacleTreeNode = new ObstacleTreeNode();

  buildAgentTree() {
    if (this.agents.length != this.simulator.getNumAgents()) {
      this.agents = this.simulator.agents;

      for (var i = 0, len = 2 * this.agents.length; i < len; i++) {
        this.agentTree.push(new AgentTreeNode());
      }
    }

    if (this.agents.length > 0) {
      this._buildAgentTreeRecursive(0, this.agents.length, 0);
    }
  }

  private _buildAgentTreeRecursive(begin: number, end: number, node: number) {
    this.agentTree[node].begin = begin;
    this.agentTree[node].end = end;
    this.agentTree[node].minX = this.agentTree[node].maxX = this.agents[begin].position.x;
    this.agentTree[node].minY = this.agentTree[node].maxY = this.agents[begin].position.y;

    for (var i = begin + 1; i < end; ++i) {
      this.agentTree[node].maxX = Math.max(this.agentTree[node].maxX, this.agents[i].position.x);
      this.agentTree[node].minX = Math.max(this.agentTree[node].minX, this.agents[i].position.x);
      this.agentTree[node].maxY = Math.max(this.agentTree[node].maxX, this.agents[i].position.y);
      this.agentTree[node].minY = Math.max(this.agentTree[node].minY, this.agents[i].position.y);
    }

    if (end - begin > this.MAXLEAF_SIZE) {
      // no leaf node
      var isVertical = this.agentTree[node].maxX - this.agentTree[node].minX > this.agentTree[node].maxY - this.agentTree[node].minY;
      var splitValue = isVertical ? 0.5 * (this.agentTree[node].maxX + this.agentTree[node].minX) : 0.5 * (this.agentTree[node].maxY + this.agentTree[node].minY);

      var left = begin;
      var right = end;

      while (left < right) {
        while (left < right && (isVertical ? this.agents[left].position.x : this.agents[left].position.y) < splitValue) {
          ++left;
        }

        while (right > left && (isVertical ? this.agents[right - 1].position.x : this.agents[right - 1].position.y) >= splitValue) {
          --right;
        }

        if (left < right) {
          var tmp = this.agents[left];
          this.agents[left] = this.agents[right - 1];
          this.agents[right - 1] = tmp;
          ++left;
          --right;
        }
      }

      var leftSize = left - begin;
      if (leftSize == 0) {
        ++leftSize;
        ++left;
        ++right;
      }

      this.agentTree[node].left = node + 1;
      this.agentTree[node].right = node + 1 + (2 * leftSize - 1);

      this._buildAgentTreeRecursive(begin, left, this.agentTree[node].left);
      this._buildAgentTreeRecursive(left, end, this.agentTree[node].right);
    }
  }

  buildObstacleTree() {
    var obstacles = this.simulator.obstacles;
    this.obstacleTree = this._buildObstacleTreeRecursive(obstacles);
  }

  private _buildObstacleTreeRecursive(obstacles: Obstacle[]) {
    if (obstacles.length == 0) {
      return null;
    } else {
      var node = new ObstacleTreeNode();
      var optimalSplit = 0;
      let minLeft = obstacles.length;
      let minRight = obstacles.length;

      for (let i = 0, len = obstacles.length; i < len; ++i) {
        let leftSize = 0;
        let rightSize = 0;

        let obstacleI1 = obstacles[i];
        let obstacleI2 = obstacleI1.next;

        for (let j = 0; j < obstacles.length; j++) {
          if (i == j) {
            continue;
          }

          let obstacleJ1 = obstacles[j];
          let obstacleJ2 = obstacleJ1.next;

          let j1LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ1.point);
          let j2LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ2.point);

          if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON) {
            ++leftSize;
          } else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON) {
            ++rightSize;
          } else {
            ++leftSize;
            ++rightSize;
          }

          var fp1 = new FloatPair(Math.max(leftSize, rightSize), Math.min(leftSize, rightSize));
          var fp2 = new FloatPair(Math.max(minLeft, minRight), Math.min(minLeft, minRight));

          if (fp1._get(fp2)) {
            break;
          }
        }

        var fp1 = new FloatPair(Math.max(leftSize, rightSize), Math.min(leftSize, rightSize));
        var fp2 = new FloatPair(Math.max(minLeft, minRight), Math.min(minLeft, minRight));

        if (fp1._mt(fp2)) {
          minLeft = leftSize;
          minRight = rightSize;
          optimalSplit = i;
        }
      }

      {
        /* Build split node. */
        let leftObstacles = [];
        for (var n = 0; n < minLeft; ++n) leftObstacles.push(null);

        let rightObstacles = [];
        for (var n = 0; n < minRight; ++n) rightObstacles.push(null);

        let leftCounter = 0;
        let rightCounter = 0;
        let i = optimalSplit;

        let obstacleI1 = obstacles[i];
        let obstacleI2 = obstacleI1.next;

        for (var j = 0; j < obstacles.length; ++j) {
          if (i == j) {
            continue;
          }

          let obstacleJ1 = obstacles[j];
          let obstacleJ2 = obstacleJ1.next;

          let j1LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ1.point);
          let j2LeftOfI = RVOMath.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ2.point);

          if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON) {
            leftObstacles[leftCounter++] = obstacles[j]
          } else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON) {
            rightObstacles[rightCounter++] = obstacles[j]
          } else {
            /* Split obstacle j. */
            let t = RVOMath.det(obstacleI2.point.minus(obstacleI1.point), obstacleJ1.point.minus(obstacleI1.point)) /
              RVOMath.det(obstacleI2.point.minus(obstacleI1.point), obstacleJ1.point.minus(obstacleJ2.point));

            var splitpoint = obstacleJ1.point.plus((obstacleJ2.point.minus(obstacleJ1.point)).scale(t));

            var newObstacle = new Obstacle();
            newObstacle.point = splitpoint;
            newObstacle.previous = obstacleJ1;
            newObstacle.next = obstacleJ2;
            newObstacle.isConvex = true;
            newObstacle.unitDir = obstacleJ1.unitDir;

            newObstacle.id = this.simulator.obstacles.length;

            this.simulator.obstacles.push(newObstacle);

            obstacleJ1.next = newObstacle;
            obstacleJ2.previous = newObstacle;

            if (j1LeftOfI > 0.0) {
              leftObstacles[leftCounter++] = obstacleJ1;
              rightObstacles[rightCounter++] = newObstacle;
            } else {
              rightObstacles[rightCounter++] = obstacleJ1;
              leftObstacles[leftCounter++] = newObstacle;
            }
          }
        }

        node.obstacle = obstacleI1;
        node.left = this._buildObstacleTreeRecursive(leftObstacles);
        node.right = this._buildObstacleTreeRecursive(rightObstacles);
        return node;
      }
    }
  }

  computeAgentNeighbors(agent: Agent, rangeSq: number) {
    this._queryAgentTreeRecursive(agent, rangeSq, 0);
  }

  computeObstacleNeighbors(agent: Agent, rangeSq: number) {
    this._queryObstacleTreeRecursive(agent, rangeSq, this.obstacleTree);
  }

  private _queryAgentTreeRecursive(agent: Agent, rangeSq: number, node: number) {
    let agentTree = this.agentTree;
    if (agentTree[node].end - agentTree[node].begin <= this.MAXLEAF_SIZE) {
      for (var i = agentTree[node].begin; i < agentTree[node].end; ++i) {
        agent.insertAgentNeighbor(this.agents[i], rangeSq);
      }
    } else {
      let distSqLeft = RVOMath.sqr(Math.max(0, agentTree[agentTree[node].left].minX - agent.position.x)) +
        RVOMath.sqr(Math.max(0, agent.position.x - agentTree[agentTree[node].left].maxX)) +
        RVOMath.sqr(Math.max(0, agentTree[agentTree[node].left].minY - agent.position.y)) +
        RVOMath.sqr(Math.max(0, agent.position.y - agentTree[agentTree[node].left].maxY));

      let distSqRight = RVOMath.sqr(Math.max(0, agentTree[agentTree[node].right].minX - agent.position.x)) +
        RVOMath.sqr(Math.max(0, agent.position.x - agentTree[agentTree[node].right].maxX)) +
        RVOMath.sqr(Math.max(0, agentTree[agentTree[node].right].minY - agent.position.y)) +
        RVOMath.sqr(Math.max(0, agent.position.y - agentTree[agentTree[node].right].maxY));

      if (distSqLeft < distSqRight) {
        if (distSqLeft < rangeSq) {
          this._queryAgentTreeRecursive(agent, rangeSq, agentTree[node].left);

          if (distSqRight < rangeSq) {
            this._queryAgentTreeRecursive(agent, rangeSq, agentTree[node].right);
          }
        }
      } else {
        if (distSqRight < rangeSq) {
          this._queryAgentTreeRecursive(agent, rangeSq, agentTree[node].right);

          if (distSqLeft < rangeSq) {
            this._queryAgentTreeRecursive(agent, rangeSq, agentTree[node].left);
          }
        }
      }

    }
  }

  // pass ref range
  private _queryObstacleTreeRecursive(agent: Agent, rangeSq: number, node: ObstacleTreeNode) {
    if (node == null) {
      return;
    } else {
      let obstacle1 = node.obstacle;
      let obstacle2 = obstacle1.next;

      let agentLeftOfLine = RVOMath.leftOf(obstacle1.point, obstacle2.point, agent.position);

      this._queryObstacleTreeRecursive(agent, rangeSq, (agentLeftOfLine >= 0 ? node.left : node.right));

      let distSqLine = RVOMath.sqr(agentLeftOfLine) / RVOMath.absSq(obstacle2.point.minus(obstacle1.point));

      if (distSqLine < rangeSq) {
        if (agentLeftOfLine < 0) {
          /*
           * Try obstacle at this node only if is on right side of
           * obstacle (and can see obstacle).
           */
          agent.insertObstacleNeighbor(node.obstacle, rangeSq);
        }

        /* Try other side of line. */
        this._queryObstacleTreeRecursive(agent, rangeSq, (agentLeftOfLine >= 0 ? node.right : node.left));
      }
    }
  }

  queryVisibility(q1: Vector2D, q2: Vector2D, radius: number): boolean {
    return this._queryVisibilityRecursive(q1, q2, radius, this.obstacleTree);
  }

  private _queryVisibilityRecursive(q1: Vector2D, q2: Vector2D, radius: number, node: ObstacleTreeNode): boolean {
    if (node == null) {
      return true;
    } else {
      var obstacle1 = node.obstacle;
      var obstacle2 = obstacle1.next;

      var q1LeftOfI = RVOMath.leftOf(obstacle1.point, obstacle2.point, q1);
      var q2LeftOfI = RVOMath.leftOf(obstacle1.point, obstacle2.point, q2);
      var invLengthI = 1.0 / RVOMath.absSq(obstacle2.point.minus(obstacle1.point));

      if (q1LeftOfI >= 0 && q2LeftOfI >= 0) {
        return this._queryVisibilityRecursive(q1, q2, radius, node.left)
          && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius)
            && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius))
            || this._queryVisibilityRecursive(q1, q2, radius, node.right));
      } else if (q1LeftOfI <= 0 && q2LeftOfI <= 0) {
        return this._queryVisibilityRecursive(q1, q2, radius, node.right)
          && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius)
            && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius))
            || this._queryVisibilityRecursive(q1, q2, radius, node.left));
      } else if (q1LeftOfI >= 0 && q2LeftOfI <= 0) {
        /* One can see through obstacle from left to right. */
        return this._queryVisibilityRecursive(q1, q2, radius, node.left)
          && this._queryVisibilityRecursive(q1, q2, radius, node.right);
      } else {
        var point1LeftOfQ = RVOMath.leftOf(q1, q2, obstacle1.point);
        var point2LeftOfQ = RVOMath.leftOf(q1, q2, obstacle2.point);
        var invLengthQ = 1.0 / RVOMath.absSq(q2.minus(q1));

        return (
          point1LeftOfQ * point2LeftOfQ >= 0
          && RVOMath.sqr(point1LeftOfQ) * invLengthQ > RVOMath.sqr(radius)
          && RVOMath.sqr(point2LeftOfQ) * invLengthQ > RVOMath.sqr(radius)
          && this._queryVisibilityRecursive(q1, q2, radius, node.left)
          && this._queryVisibilityRecursive(q1, q2, radius, node.right)
        )
      }
    }
  }
}

class FloatPair {

  a = 0;
  b = 0;

  constructor(a: number = 0, b: number = 0) {
    this.a = a;
    this.b = b;
  }

  _mt(rhs: FloatPair): boolean {
    return this.a < rhs.a || !(rhs.a < this.a) && this.b < rhs.b;
  }

  _met(rhs: FloatPair): boolean {
    return (this.a == rhs.a && this.b == rhs.b) || this._mt(rhs);
  }

  //greater-than
  _gt(rhs: FloatPair): boolean {
    return !this._met(rhs);
  }

  //greater-or-equal-than
  _get(rhs: FloatPair): boolean {
    return !this._mt(rhs);
  }
}


class AgentTreeNode {
  begin = 0;
  end = 0;
  left = 0;
  maxX = 0;
  maxY = 0;
  minX = 0;
  minY = 0;
  right = 0;
}

class ObstacleTreeNode {
  obstacle: Obstacle;
  left: ObstacleTreeNode;
  right: ObstacleTreeNode;
}
