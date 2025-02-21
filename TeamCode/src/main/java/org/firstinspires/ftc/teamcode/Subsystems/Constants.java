package org.firstinspires.ftc.teamcode.Subsystems;

public final class Constants {
    private Constants(){}

    /* ARM POSITIONS */
            public static class Arm {
                    public static final double ARM_STOW = -35;

                    // GROUND GRAB HOVER
                    public static final double ARM_GROUNDGRAB = -22;

                    // GROUND GRAB PICK
                    public static final double ARM_GROUNDGRAB_PICK = -26;

                    // SPECIMENS PICK
                    public static final double ARM_SPECIMENS = -25; /* PENDING TEST */

                    // BASKETS
                    public static final double ARM_LOWBASKET = 90;
                    public static final double ARM_HIGHBASKET = 90;

                    // CHAMBERS
                    public static final double ARM_LOWCHAMBER = 0;
                    public static final double ARM_HIGHCHAMBER = 0;
            }

    /* INTAKE POSITIONS */
            public static class Intake {
                public static final double INTAKE_STOW = 0.0;
                public static final double INTAKE_OPEN = 1.0;
            }

    /* ELEVATOR POSITIONS */
            public static class Elevator {
                public static final double ELEVATOR_STOW = 0;

                public static final double ELEVATOR_HOVER_GROUNDGRAB = 48; //18.9 in


                public static final double ELEVATOR_CLIMB = 5;

              /*
                Clarification:
                 The following distances may seem to exceed the maximum length stipulated in rule R104, however,
                 the distances are with the arm at a high angle thus, the total horizontal expansion limit
                 is still acknowledged as per rule R104. This was calculated using trigonometry with the following formula.
                        HorizontalExtension = cosine(armAngle) * ElevatorExtension
               */

                // BASKETS
                public static final double ELEVATOR_LOWBASKET = 32;//23 inches of extension
                public static final double ELEVATOR_HIGHBASKET = 63;//40 inches of extension

                // CHAMBERS
                public static final double ELEVATOR_LOWCHAMBER = 0; // 0 inches of extension
                public static final double ELEVATOR_HIGHCHAMBER = 25;//

            }

    /* WRIST POSITIONS */
            public static class Wrist {
                // BASKETS
                public static final double WRIST_HIGHBASKET = 0.3;
                public static final double WRIST_LOWBASKET = 0;

                // GROUND GRAB
                public static final double WRIST_GROUNDGRAB = 0.9;

                // CHAMBERS
                public static final double WRIST_HIGHCHAMBER = 0;
                public static final double WRIST_LOWCHAMBER = 0;

                // TEST POSITIONS
                public static final double WRIST_SHORT = 0.1; //((Arm.ARM_SHORT_GROUNDGRAB + 90 + 11)/180)
                public static final double WRIST_MEDIUM = 0.5; //((Arm.ARM_MEDIUM_GROUNDGRAB + 90 + 11)/180);
                public static final double WRIST_LONG = 0.9; // (Arm.ARM_LONG_GROUNDGRAB + 90 + 11)/180);

            }
}
