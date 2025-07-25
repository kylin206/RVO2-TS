import Obstacle from '../Obstacle';
import Vector2D from '../Vector2D';

describe('Obstacle', () => {
  describe('properties', () => {
    it('should initialize with default values', () => {
      const obstacle = new Obstacle();
      expect(obstacle.point.x).toBe(0);
      expect(obstacle.point.y).toBe(0);
      expect(obstacle.unitDir.x).toBe(0);
      expect(obstacle.unitDir.y).toBe(0);
      expect(obstacle.isConvex).toBe(false);
      expect(obstacle.id).toBe(0);
      expect(obstacle.previous).toBeUndefined();
      expect(obstacle.next).toBeUndefined();
    });

    it('should allow setting properties', () => {
      const obstacle = new Obstacle();
      const point = new Vector2D(1, 2);
      const unitDir = new Vector2D(0, 1);
      
      obstacle.point = point;
      obstacle.unitDir = unitDir;
      obstacle.isConvex = true;
      obstacle.id = 1;

      expect(obstacle.point).toBe(point);
      expect(obstacle.unitDir).toBe(unitDir);
      expect(obstacle.isConvex).toBe(true);
      expect(obstacle.id).toBe(1);
    });
  });

  describe('linking', () => {
    it('should link obstacles correctly', () => {
      const obstacle1 = new Obstacle();
      const obstacle2 = new Obstacle();
      const obstacle3 = new Obstacle();

      // Create a circular linked list
      obstacle1.next = obstacle2;
      obstacle2.previous = obstacle1;
      obstacle2.next = obstacle3;
      obstacle3.previous = obstacle2;
      obstacle3.next = obstacle1;
      obstacle1.previous = obstacle3;

      // Verify links
      expect(obstacle1.next).toBe(obstacle2);
      expect(obstacle2.next).toBe(obstacle3);
      expect(obstacle3.next).toBe(obstacle1);
      expect(obstacle1.previous).toBe(obstacle3);
      expect(obstacle2.previous).toBe(obstacle1);
      expect(obstacle3.previous).toBe(obstacle2);
    });
  });
});