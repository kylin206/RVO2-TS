import Line from '../Line';
import Vector2D from '../Vector2D';

describe('Line', () => {
  describe('constructor', () => {
    it('should create line with default zero vectors', () => {
      const line =  new Line(new Vector2D(), new Vector2D());
      expect(line.point.x).toBe(0);
      expect(line.point.y).toBe(0);
      expect(line.direction.x).toBe(0);
      expect(line.direction.y).toBe(0);
    });

    it('should create line with given point and direction', () => {
      const point = new Vector2D(1, 2);
      const direction = new Vector2D(3, 4);
      const line = new Line(point, direction);
      expect(line.point).toBe(point);
      expect(line.direction).toBe(direction);
    });
  });
});