import RVOMath from "./RVOMath";
import Simulator from "./Simulator";
import Vector2D from "./Vector2D";
import Obstacle from "./Obstacle";
import Line from "./Line";

export default class Agent {
  public id = 0;
  simulator: Simulator;
  agentNeighbors = []; //  new List<KeyValuePair<float, Agent>>()
  maxNeighbors = 0;
  maxSpeed = 0.0;
  neighborDist = 0.0;
  private _newVelocity: Vector2D;
  obstaclNeighbors: KeyValuePair[] = []; // new List<KeyValuePair<float, Obstacle>>()
  orcaLines: Line[] = [];
  position: Vector2D;

  prefVelocity: Vector2D;

  radius = 0.0;
  timeHorizon = 0.0;
  timeHorizonObst = 0.0;
  velocity: Vector2D;

  computeNeighbors() {
    this.obstaclNeighbors = [];
    var rangeSq = RVOMath.sqr(this.timeHorizonObst * this.maxSpeed + this.radius);
    this.simulator.kdTree.computeObstacleNeighbors(this, rangeSq);

    this.agentNeighbors = [];
    if (this.maxNeighbors > 0) {
      rangeSq = RVOMath.sqr(this.neighborDist);
      this.simulator.kdTree.computeAgentNeighbors(this, rangeSq);
    }
  }

  /* Search for the best new velocity. */
  computeNewVelocity() {
    this.orcaLines.length = 0;
    let orcaLines = this.orcaLines;
    const invTimeHorizonObst = 1.0 / this.timeHorizonObst;

    /* Create obstacle ORCA lines. */
    for (var i = 0; i < this.obstaclNeighbors.length; ++i) {
      let obstacle1: Obstacle = this.obstaclNeighbors[i].value;
      let obstacle2 = obstacle1.next;

      let relativePosition1 = obstacle1.point.minus(this.position)
      let relativePosition2 = obstacle2.point.minus(this.position)

      /*
       * Check if velocity obstacle of obstacle is already taken care of by
       * previously constructed obstacle ORCA lines.
       */
      let alreadyCovered = false

      for (var j = 0; j < orcaLines.length; ++j) {
        if (RVOMath.det(relativePosition1.scale(invTimeHorizonObst).minus(orcaLines[j].point), orcaLines[j].direction) - invTimeHorizonObst * this.radius >= -RVOMath.RVO_EPSILON && RVOMath.det(relativePosition2.scale(invTimeHorizonObst).minus(orcaLines[j].point), orcaLines[j].direction) - invTimeHorizonObst * this.radius >= -RVOMath.RVO_EPSILON) {
          alreadyCovered = true
          break
        }
      }

      if (alreadyCovered) {
        continue
      }

      /* Not yet covered. Check for collisions. */

      let distSq1 = RVOMath.absSq(relativePosition1)
      let distSq2 = RVOMath.absSq(relativePosition2)

      let radiusSq = RVOMath.sqr(this.radius)

      let obstacleVector = obstacle2.point.minus(obstacle1.point)
      let s = relativePosition1.scale(-1).multiply(obstacleVector) / RVOMath.absSq(obstacleVector); //  (-relativePosition1 * obstacleVector) / RVOMath.absSq(obstacleVector)
      let distSqLine = RVOMath.absSq(relativePosition1.scale(-1).minus(obstacleVector.scale(s))); // RVOMath.absSq(-relativePosition1 - s * obstacleVector)

      var line = new Line();

      if (s < 0 && distSq1 <= radiusSq) {
        /* Collision with left vertex. Ignore if non-convex. */
        if (obstacle1.isConvex) {
          line.point = new Vector2D(0, 0)
          line.direction = RVOMath.normalize(new Vector2D(-relativePosition1.y, relativePosition1.x))
          orcaLines.push(line)
        }
        continue
      } else if (s > 1 && distSq2 <= radiusSq) {
        /* Collision with right vertex. Ignore if non-convex
         * or if it will be taken care of by neighoring obstace */
        if (obstacle2.isConvex && RVOMath.det(relativePosition2, obstacle2.unitDir) >= 0) {
          line.point = new Vector2D(0, 0)
          line.direction = RVOMath.normalize(new Vector2D(-relativePosition2.y, relativePosition2.x))
          orcaLines.push(line)
        }
        continue
      } else if (s >= 0 && s < 1 && distSqLine <= radiusSq) {
        /* Collision with obstacle segment. */
        line.point = new Vector2D(0, 0)
        line.direction = obstacle1.unitDir.scale(-1)
        orcaLines.push(line)
        continue
      }

      /*
       * No collision.
       * Compute legs. When obliquely viewed, both legs can come from a single
       * vertex. Legs extend cut-off line when nonconvex vertex.
       */

      var leftLegDirection, rightLegDirection

      if (s < 0 && distSqLine <= radiusSq) {
        /*
         * Obstacle viewed obliquely so that left vertex
         * defines velocity obstacle.
         */
        if (!obstacle1.isConvex) {
          /* Ignore obstacle. */
          continue
        }

        obstacle2 = obstacle1

        let leg1 = Math.sqrt(distSq1 - radiusSq)
        leftLegDirection = (new Vector2D(relativePosition1.x * leg1 - relativePosition1.y * this.radius, relativePosition1.x * this.radius + relativePosition1.y * leg1)).scale(1 / distSq1)
        rightLegDirection = (new Vector2D(relativePosition1.x * leg1 + relativePosition1.y * this.radius, -relativePosition1.x * this.radius + relativePosition1.y * leg1)).scale(1 / distSq1)
      } else if (s > 1 && distSqLine <= radiusSq) {
        /*
         * Obstacle viewed obliquely so that
         * right vertex defines velocity obstacle.
         */
        if (!obstacle2.isConvex) {
          /* Ignore obstacle. */
          continue
        }

        obstacle1 = obstacle2

        let leg2 = Math.sqrt(distSq2 - radiusSq)
        leftLegDirection = (new Vector2D(relativePosition2.x * leg2 - relativePosition2.y * this.radius, relativePosition2.x * this.radius + relativePosition2.y * leg2)).scale(1 / distSq2)
        rightLegDirection = (new Vector2D(relativePosition2.x * leg2 + relativePosition2.y * this.radius, -relativePosition2.x * this.radius + relativePosition2.y * leg2)).scale(1 / distSq2)
      } else {
        /* Usual situation. */
        if (obstacle1.isConvex) {
          let leg1 = Math.sqrt(distSq1 - radiusSq)
          leftLegDirection = (new Vector2D(relativePosition1.x * leg1 - relativePosition1.y * this.radius, relativePosition1.x * this.radius + relativePosition1.y * leg1)).scale(1 / distSq1)
        } else {
          /* Left vertex non-convex; left leg extends cut-off line. */
          leftLegDirection = obstacle1.unitDir.scale(-1)
        }

        if (obstacle2.isConvex) {
          let leg2 = Math.sqrt(distSq2 - radiusSq)
          rightLegDirection = (new Vector2D(relativePosition2.x * leg2 + relativePosition2.y * this.radius, -relativePosition2.x * this.radius + relativePosition2.y * leg2)).scale(1 / distSq2)
        } else {
          /* Right vertex non-convex; right leg extends cut-off line. */
          rightLegDirection = obstacle1.unitDir
        }
      }

      /*
       * Legs can never point into neighboring edge when convex vertex,
       * take cutoff-line of neighboring edge instead. If velocity projected on
       * "foreign" leg, no constraint is added.
       */

      let leftNeighbor = obstacle1.previous;

      let isLeftLegForeign = false;
      let isRightLegForeign = false;

      if (obstacle1.isConvex && RVOMath.det(leftLegDirection, leftNeighbor.unitDir.scale(-1)) >= 0.0) {
        /* Left leg points into obstacle. */
        leftLegDirection = leftNeighbor.unitDir.scale(-1)
        isLeftLegForeign = true
      }

      if (obstacle2.isConvex && RVOMath.det(rightLegDirection, obstacle2.unitDir) <= 0.0) {
        /* Right leg points into obstacle. */
        rightLegDirection = obstacle2.unitDir
        isRightLegForeign = true
      }

      /* Compute cut-off centers. */
      let leftCutoff = obstacle1.point.minus(this.position).scale(invTimeHorizonObst)
      let rightCutoff = obstacle2.point.minus(this.position).scale(invTimeHorizonObst)
      let cutoffVec = rightCutoff.minus(leftCutoff)

      /* Project current velocity on velocity obstacle. */

      /* Check if current velocity is projected on cutoff circles. */
      let t = obstacle1 == obstacle2 ? 0.5 : this.velocity.minus(leftCutoff).multiply(cutoffVec) / RVOMath.absSq(cutoffVec)
      let tLeft = this.velocity.minus(leftCutoff).multiply(leftLegDirection)
      let tRight = this.velocity.minus(rightCutoff).multiply(rightLegDirection)

      if ((t < 0.0 && tLeft < 0.0) || (obstacle1 == obstacle2 && tLeft < 0.0 && tRight < 0.0)) {
        /* Project on left cut-off circle. */
        let unitW = RVOMath.normalize(this.velocity.minus(leftCutoff))

        line.direction = new Vector2D(unitW.y, -unitW.x)
        line.point = leftCutoff.plus(unitW.scale(this.radius * invTimeHorizonObst))
        orcaLines.push(line)
        continue
      } else if (t > 1.0 && tRight < 0.0) {
        /* Project on right cut-off circle. */
        let unitW = RVOMath.normalize(this.velocity.minus(rightCutoff))

        line.direction = new Vector2D(unitW.y, -unitW.x)
        line.point = rightCutoff.plus(unitW.scale(this.radius * invTimeHorizonObst))
        orcaLines.push(line)
        continue
      }

      /*
       * Project on left leg, right leg, or cut-off line, whichever is closest
       * to velocity.
       */
      let distSqCutoff = ((t < 0.0 || t > 1.0 || obstacle1 == obstacle2) ? Infinity : RVOMath.absSq(this.velocity.minus(cutoffVec.scale(t).plus(leftCutoff))))
      let distSqLeft = ((tLeft < 0.0) ? Infinity : RVOMath.absSq(this.velocity.minus(leftLegDirection.scale(tLeft).plus(leftCutoff))))
      let distSqRight = ((tRight < 0.0) ? Infinity : RVOMath.absSq(this.velocity.minus(rightLegDirection.scale(tRight).plus(rightCutoff))))

      if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
        /* Project on cut-off line. */
        line.direction = obstacle1.unitDir.scale(-1)
        var aux = new Vector2D(-line.direction.y, line.direction.x)
        line.point = aux.scale(this.radius * invTimeHorizonObst).plus(leftCutoff)
        orcaLines.push(line)
        continue
      } else if (distSqLeft <= distSqRight) {
        /* Project on left leg. */
        if (isLeftLegForeign) {
          continue
        }

        line.direction = leftLegDirection
        var aux = new Vector2D(-line.direction.y, line.direction.x)
        line.point = aux.scale(this.radius * invTimeHorizonObst).plus(leftCutoff)
        orcaLines.push(line)
        continue
      } else {
        /* Project on right leg. */
        if (isRightLegForeign) {
          continue
        }

        line.direction = rightLegDirection.scale(-1)
        var aux = new Vector2D(-line.direction.y, line.direction.x)
        line.point = aux.scale(this.radius * invTimeHorizonObst).plus(leftCutoff)
        orcaLines.push(line)
        continue
      }
    }

    var numObstLines = orcaLines.length

    var invTimeHorizon = 1.0 / this.timeHorizon

    /* Create agent ORCA lines. */
    for (var i = 0; i < this.agentNeighbors.length; ++i) {
      var other = this.agentNeighbors[i].value

      let relativePosition = other.position.minus(this.position)
      let relativeVelocity = this.velocity.minus(other.velocity)
      let distSq = RVOMath.absSq(relativePosition)
      let combinedRadius = this.radius + other.radius
      let combinedRadiusSq = RVOMath.sqr(combinedRadius)

      var line = new Line(); // Line
      var u: Vector2D;

      if (distSq > combinedRadiusSq) {
        /* No collision. */
        let w = relativeVelocity.minus(relativePosition.scale(invTimeHorizon));; // Vector
        /* Vector from cutoff center to relative velocity. */
        let wLengthSq = RVOMath.absSq(w)

        let dotProduct1 = w.multiply(relativePosition)

        if (dotProduct1 < 0.0 && RVOMath.sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
          /* Project on cut-off circle. */
          let wLength = Math.sqrt(wLengthSq);
          let unitW = w.scale(1 / wLength)

          line.direction = new Vector2D(unitW.y, -unitW.x)
          u = unitW.scale(combinedRadius * invTimeHorizon - wLength)
        } else {
          /* Project on legs. */
          let leg = Math.sqrt(distSq - combinedRadiusSq)

          if (RVOMath.det(relativePosition, w) > 0.0) {
            /* Project on left leg. */
            var aux = new Vector2D(relativePosition.x * leg - relativePosition.y * combinedRadius, relativePosition.x * combinedRadius + relativePosition.y * leg)
            line.direction = aux.scale(1 / distSq)
          } else {
            /* Project on right leg. */
            var aux = new Vector2D(relativePosition.x * leg + relativePosition.y * combinedRadius, -relativePosition.x * combinedRadius + relativePosition.y * leg)
            line.direction = aux.scale(-1 / distSq)
          }

          let dotProduct2 = relativeVelocity.multiply(line.direction)

          u = line.direction.scale(dotProduct2).minus(relativeVelocity)
        }
      } else {
        /* Collision. Project on cut-off circle of time timeStep. */
        let invTimeStep = 1.0 / this.simulator.timeStep

        /* Vector from cutoff center to relative velocity. */
        const w = relativeVelocity.minus(relativePosition.scale(invTimeStep))

        let wLength = RVOMath.abs(w)
        let unitW = w.scale(1 / wLength);

        line.direction = new Vector2D(unitW.y, -unitW.x)
        u = unitW.scale(combinedRadius * invTimeStep - wLength)
      }

      line.point = u.scale(0.5).plus(this.velocity)
      orcaLines.push(line)
    }

    let lineFail = this._linearProgram2(orcaLines, this.maxSpeed, this.prefVelocity, false);

    if (lineFail < orcaLines.length) {
      this._linearProgram3(orcaLines, numObstLines, lineFail, this.maxSpeed);
    }
  }

  insertAgentNeighbor(agent: Agent, rangeSq: number) {
    if (this != agent) {
      var distSq = RVOMath.absSq(this.position.minus(agent.position))

      if (distSq < rangeSq) {
        if (this.agentNeighbors.length < this.maxNeighbors) {
          this.agentNeighbors.push(new KeyValuePair(distSq, agent))
        }
        var i = this.agentNeighbors.length - 1
        while (i != 0 && distSq < this.agentNeighbors[i - 1].key) {
          this.agentNeighbors[i] = this.agentNeighbors[i - 1]
          --i
        }
        this.agentNeighbors[i] = new KeyValuePair(distSq, agent)

        if (this.agentNeighbors.length == this.maxNeighbors) {
          rangeSq = this.agentNeighbors[this.agentNeighbors.length - 1].key
        }
      }
    }
  }

  insertObstacleNeighbor(obstacle: Obstacle, rangeSq: number) {
    let nextObstacle = obstacle.next;

    let distSq = RVOMath.distSqPointLineSegment(obstacle.point, nextObstacle.point, this.position)

    if (distSq < rangeSq) {
      this.obstaclNeighbors.push(new KeyValuePair(distSq, obstacle))

      let i = this.obstaclNeighbors.length - 1
      while (i != 0 && distSq < this.obstaclNeighbors[i - 1].key) {
        this.obstaclNeighbors[i] = this.obstaclNeighbors[i - 1]
        --i
      }
      this.obstaclNeighbors[i] = new KeyValuePair(distSq, obstacle)
    }
  }

  update() {
    // var rnd = new Vector2D(Math.random() * 0.1 - 0.05, Math.random() * 0.1 - 0.05)
    // this.velocity = this.newVelocity.plus(rnd)
    this.velocity = this._newVelocity;
    this.position = this.position.plus(this._newVelocity.scale(this.simulator.timeStep))
  }

  private _linearProgram1(lines: Line[],
    lineNo: number,
    radius: number,
    optVelocity: Vector2D,
    directionOpt: boolean): boolean {

    var dotProduct = lines[lineNo].point.multiply(lines[lineNo].direction)
    var discriminant = RVOMath.sqr(dotProduct) + RVOMath.sqr(radius) - RVOMath.absSq(lines[lineNo].point)

    if (discriminant < 0.0) {
      /* Max speed circle fully invalidates line lineNo. */
      return false;
    }

    var sqrtDiscriminant = Math.sqrt(discriminant);
    var tLeft = -dotProduct - sqrtDiscriminant;
    var tRight = -dotProduct + sqrtDiscriminant;

    for (var i = 0; i < lineNo; ++i) {
      var denominator = RVOMath.det(lines[lineNo].direction, lines[i].direction);
      var numerator = RVOMath.det(lines[i].direction, lines[lineNo].point.minus(lines[i].point));

      if (Math.abs(denominator) <= RVOMath.RVO_EPSILON) {
        /* Lines lineNo and i are (almost) parallel. */
        if (numerator < 0.0) {
          return false;
        } else {
          continue;
        }
      }

      var t = numerator / denominator;

      if (denominator >= 0.0) {
        /* Line i bounds line lineNo on the right. */
        tRight = Math.min(tRight, t);
      } else {
        /* Line i bounds line lineNo on the left. */
        tLeft = Math.max(tLeft, t);
      }

      if (tLeft > tRight) {
        return false;
      }
    }

    if (directionOpt) {
      if (optVelocity.multiply(lines[lineNo].direction) > 0.0) {
        // Take right extreme
        this._newVelocity = lines[lineNo].direction.scale(tRight).plus(lines[lineNo].point);
      } else {
        // Take left extreme.
        this._newVelocity = lines[lineNo].direction.scale(tLeft).plus(lines[lineNo].point);
      }
    } else {
      // Optimize closest point
      t = lines[lineNo].direction.multiply(optVelocity.minus(lines[lineNo].point));

      if (t < tLeft) {
        this._newVelocity = lines[lineNo].direction.scale(tLeft).plus(lines[lineNo].point);
      } else if (t > tRight) {
        this._newVelocity = lines[lineNo].direction.scale(tRight).plus(lines[lineNo].point);
      } else {
        this._newVelocity = lines[lineNo].direction.scale(t).plus(lines[lineNo].point);
      }
    }

    // TODO ugly hack by palmerabollo
    if (isNaN(this._newVelocity.x) || isNaN(this._newVelocity.y)) {
      return false;
    }

    return true;
  }

  private _linearProgram2(lines: Line[],
    radius: number,
    optVelocity: Vector2D,
    directionOpt: boolean): number {
    if (directionOpt) {
      /*
       * Optimize direction. Note that the optimization velocity is of unit
       * length in this case.
       */
      this._newVelocity = optVelocity.scale(radius);
    } else if (RVOMath.absSq(optVelocity) > RVOMath.sqr(radius)) {
      /* Optimize closest point and outside circle. */
      this._newVelocity = RVOMath.normalize(optVelocity).scale(radius);
    } else {
      /* Optimize closest point and inside circle. */
      this._newVelocity = optVelocity;
    }

    for (var i = 0; i < lines.length; ++i) {
      if (RVOMath.det(lines[i].direction, lines[i].point.minus(this._newVelocity)) > 0.0) {
        /* Result does not satisfy constraint i. Compute new optimal result. */
        var tempResult = this._newVelocity;
        if (!this._linearProgram1(lines, i, this.radius, optVelocity, directionOpt)) {
          this._newVelocity = tempResult;
          return i;
        }
      }
    }

    return lines.length;
  }

  private _linearProgram3(lines: Line[], numObstLines: number, beginLine: number, radius: number) {
    var distance = 0.0;

    for (var i = beginLine; i < lines.length; ++i) {
      if (RVOMath.det(lines[i].direction, lines[i].point.minus(this._newVelocity)) > distance) {
        /* Result does not satisfy constraint of line i. */
        //std::vector<Line> projLines(lines.begin(), lines.begin() + numObstLines)
        let projLines = []; // new List<Line>()
        for (var ii = 0; ii < numObstLines; ++ii) {
          projLines.push(lines[ii]);
        }

        for (var j = numObstLines; j < i; ++j) {
          var line = new Line();

          let determinant = RVOMath.det(lines[i].direction, lines[j].direction);

          if (Math.abs(determinant) <= RVOMath.RVO_EPSILON) {
            /* Line i and line j are parallel. */
            if (lines[i].direction.multiply(lines[j].direction) > 0.0) {
              /* Line i and line j point in the same direction. */
              continue;
            } else {
              /* Line i and line j point in opposite direction. */
              line.point = lines[i].point.plus(lines[j].point).scale(0.5);
            }
          } else {
            var aux = lines[i].direction.scale(RVOMath.det(lines[j].direction, lines[i].point.minus(lines[j].point)) / determinant);
            line.point = lines[i].point.plus(aux);
          }

          line.direction = RVOMath.normalize(lines[j].direction.minus(lines[i].direction));
          projLines.push(line);
        }

        var tempResult = this._newVelocity;
        if (this._linearProgram2(projLines, radius, new Vector2D(-lines[i].direction.y, lines[i].direction.x), true) < projLines.length) {
          /* This should in principle not happen.  The result is by definition
           * already in the feasible region of this linear program. If it fails,
           * it is due to small floating point error, and the current result is
           * kept.
           */
          this._newVelocity = tempResult;
        }

        distance = RVOMath.det(lines[i].direction, lines[i].point.minus(this._newVelocity));
      }
    }
  }
}


class KeyValuePair {
  key;
  value;

  constructor(key, value) {
    this.key = key
    this.value = value
  }
}
