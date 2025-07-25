import RVOMath from '../RVOMath';
import Vector2D from '../Vector2D';

describe('RVOMath', () => {
  describe('constants', () => {
    it('should have correct epsilon value', () => {
      expect(RVOMath.RVO_EPSILON).toBe(0.01);
    });
  });

  describe('vector operations', () => {
    it('should calculate squared absolute value of vector', () => {
      const vector = new Vector2D(3, 4);
      expect(RVOMath.absSq(vector)).toBe(25); // 3^2 + 4^2
    });

    it('should normalize vector', () => {
      const vector = new Vector2D(3, 4);
      const normalized = RVOMath.normalize(vector);
      expect(normalized.x).toBeCloseTo(0.6);
      expect(normalized.y).toBeCloseTo(0.8);
    });

    it('should calculate absolute value of vector', () => {
      const vector = new Vector2D(3, 4);
      expect(RVOMath.abs(vector)).toBe(5);
    });
  });

  describe('geometric calculations', () => {
    it('should calculate squared distance from point to line segment', () => {
      const a = new Vector2D(0, 0); // Line segment start
      const b = new Vector2D(4, 0); // Line segment end
      const c = new Vector2D(2, 2); // Point

      // Point above middle of line segment
      expect(RVOMath.distSqPointLineSegment(a, b, c)).toBe(4);

      // Point before line segment start
      const d = new Vector2D(-2, 0);
      expect(RVOMath.distSqPointLineSegment(a, b, d)).toBe(4); // (-2-0)^2 + (0-0)^2 = 4

      // Point after line segment end
      const e = new Vector2D(6, 0);
      expect(RVOMath.distSqPointLineSegment(a, b, e)).toBe(16); // (6-4)^2 + (0-0)^2 = 16
    });

    it('should calculate determinant of two vectors', () => {
      const v1 = new Vector2D(1, 2);
      const v2 = new Vector2D(3, 4);
      // det = 1*4 - 2*3 = -2
      expect(RVOMath.det(v1, v2)).toBe(-2);
    });

    it('should determine if point is left of line', () => {
      const a = new Vector2D(0, 0); // Line start
      const b = new Vector2D(4, 0); // Line end

      // Point above line (left)
      const c = new Vector2D(2, 2);
      expect(RVOMath.leftOf(a, b, c)).toBeGreaterThan(0);

      // Point below line (right)
      const d = new Vector2D(2, -2);
      expect(RVOMath.leftOf(a, b, d)).toBeLessThan(0);

      // Point on line
      const e = new Vector2D(2, 0);
      expect(Math.abs(RVOMath.leftOf(a, b, e))).toBeLessThan(RVOMath.RVO_EPSILON);
    });
  });

  describe('utility functions', () => {
    it('should calculate square of number', () => {
      expect(RVOMath.sqr(3)).toBe(9);
      expect(RVOMath.sqr(-4)).toBe(16);
      expect(RVOMath.sqr(0)).toBe(0);
    });
  });
});