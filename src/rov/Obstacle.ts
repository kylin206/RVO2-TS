
import Vector2D from "./Vector2D";


export default class Obstacle {
  point: Vector2D = Vector2D.ZERO;
  unitDir: Vector2D = Vector2D.ZERO;
  isConvex: boolean = false;
  id = 0;
  previous: Obstacle;
  next: Obstacle;
}
