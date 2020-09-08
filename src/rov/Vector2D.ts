export default class Vector2D {

  static ZERO: Vector2D = new Vector2D();

  x = 0;
  y = 0;

  constructor(x: number = 0, y: number = 0) {
    this.x = x;
    this.y = y;
  }

  plus(vector: Vector2D): Vector2D {
    return new Vector2D(this.x + vector.x, this.y + vector.y);
  }

  //subtract
  minus(vector: Vector2D): Vector2D {
    return new Vector2D(this.x - vector.x, this.y - vector.y);
  }

  multiply(vector: Vector2D): number {
    return this.x * vector.x + this.y * vector.y;
  }

  scale(k: number): Vector2D {
    return new Vector2D(this.x * k, this.y * k);
  }

  normalize(): Vector2D {
    return this.scale(1 / this.abs());
  }

  absSq(): number {
    return this.multiply(this);
  }

  abs(): number {
    return Math.sqrt(this.absSq());
  }

  clone(): Vector2D {
    return new Vector2D(this.x, this.y);
  }

}
