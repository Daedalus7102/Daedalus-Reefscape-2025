package frc.robot.Utilities;

public class PositionEnums {
    public enum AlgaeScorePositions {
        HOME(0,0,0),
        FLOOR_INTAKE(0,0,1),
        PROCCESOR_EJECT(0,0,2),
        BETWEEN_L2_AND_L3Position(0, 0, 3),
        BETWEEN_L3_AND_L4Position(0, 0, 4);
        

        private double elevatorPosition;
        private double algaeIntakePosition;
        private int positionIndex;
        
        AlgaeScorePositions(double elevatorPosition, double algaeIntakePosition, int positionIndex) {
            this.elevatorPosition = elevatorPosition;
            this.algaeIntakePosition = algaeIntakePosition;
            this.positionIndex = positionIndex;
        }

        public double getElevatorPosition() {
            return this.elevatorPosition;        
        }

        public double getAlgaeIntakePosition() {
            return this.algaeIntakePosition;        
        }

        public int getPositionIndex() {
            return this.positionIndex;
        }

        public AlgaeScorePositions indexToPosition(int index) {
            index = Clamp.clamp(index, 0, 4);
            switch (index) {
                case 0:
                    return HOME;
                case 1:
                    return FLOOR_INTAKE;
                case 2:
                    return PROCCESOR_EJECT;
                case 3:
                    return BETWEEN_L2_AND_L3Position;
                case 4:
                    return BETWEEN_L3_AND_L4Position;
                default:
                    return HOME;
            }
        }

        public AlgaeScorePositions sumPosition(AlgaeScorePositions position, int sum) {
            sum += position.getPositionIndex();
            sum = Clamp.clamp(sum, 0, 4);
            
            return indexToPosition(sum);
        }
    }
    
    public enum CoralScorePositions {
        READ_REEF_APRILTAG(0,0,0),
        HOME(0,0,0),
        INTAKE(0,0,0),
        L1(0,0,1),
        L2(0,0,2),
        L3(0,0,3),
        L4(0,0,4);

        private double elevatorPosition;
        private double coralIntakePosition;
        private int positionIndex;
        
        CoralScorePositions (double elevatorPosition, double coralIntakePosition, int positionIndex) {
            this.elevatorPosition = elevatorPosition;
            this.coralIntakePosition = coralIntakePosition;
            this.positionIndex = positionIndex;
        }

        public double getElevatorPosition() {
            return this.elevatorPosition;        
        }

        public double getCoralIntakePosition() {
            return this.coralIntakePosition;        
        }

        public int getPositionIndex() {
            return this.positionIndex;
        }

        public CoralScorePositions indexToPosition(int index) {
            index = Clamp.clamp(index, 0, 4);
            switch (index) {
                case 0:
                    return HOME;
                case 1:
                    return L1;
                case 2:
                    return L2;
                case 3:
                    return L3;
                case 4:
                    return L4;
                default:
                    return HOME;
            }
        }

        public CoralScorePositions sumPosition(AlgaeScorePositions position, int sum) {
            sum += position.getPositionIndex();
            sum = Clamp.clamp(sum, 0, 4);
            
            return indexToPosition(sum);

        }
    }
}
