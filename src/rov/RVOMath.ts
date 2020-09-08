
import Vector2D from "./Vector2D";

export default class RVOMath {
  static RVO_EPSILON = 0.01;

  static absSq(v: Vector2D) {
    return v.multiply(v);
  }

  static normalize(v: Vector2D) {
    return v.scale(1 / RVOMath.abs(v)) // v / abs(v)
  }

  static distSqPointLineSegment(a, b, c) {
    var aux1 = c.minus(a);
    var aux2 = b.minus(a);

    // r = ((c - a) * (b - a)) / absSq(b - a)
    var r = aux1.multiply(aux2) / RVOMath.absSq(aux2);

    if (r < 0) {
      return RVOMath.absSq(aux1); // absSq(c - a)
    } else if (r > 1) {
      return RVOMath.absSq(aux2);// absSq(c - b)
    } else {
      return RVOMath.absSq(c.minus(a.plus(aux2.scale(r))));// absSq(c - (a + r * (b - a)))
    }
  }

  static sqr(p: number): number {
    return p * p;
  }

  static det(v1: Vector2D, v2: Vector2D): number {
    return v1.x * v2.y - v1.y * v2.x;
  }

  static abs(v): number {
    return Math.sqrt(RVOMath.absSq(v));
  }

  static leftOf(a: Vector2D, b: Vector2D, c: Vector2D): number {
    return RVOMath.det(a.minus(c), b.minus(a));
  }
}