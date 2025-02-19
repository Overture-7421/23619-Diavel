package org.firstinspires.ftc.teamcode.Subsystems;

public final class Constants {
    private Constants(){}

    public static class Arm {
        /* ARM POSITIONS */
            public static final double ARM_STOW = -33;

            // GROUND GRAB
                // GROUND GRAB HOVER
                public static final double ARM_GROUNDGRAB = -22;

                // GROUND GRAB PICK
                public static final double ARM_GROUNDGRAB_PICK = -24;



            public static final double ARM_SPECIMENS = -25; /* PENDING TEST */

            // BASKETS
            public static final double ARM_LOWBASKET = 100;
            public static final double ARM_HIGHBASKET = 100;

            // CHAMBERS
            public static final double ARM_LOWCHAMBER = 0;
            public static final double ARM_HIGHCHAMBER = 0;// 15 si se hace de abajo-arriba y 90 si se hace de arriba-abajo
    }

    public static class Intake {
        public static final double INTAKE_STOW = 0.0;
        public static final double INTAKE_OPEN = 1.0;
    }

    /* ELEVATOR POSITIONS */
            public static class Elevator {
                public static final double ELEVATOR_STOW = 0;

                //ELEVATOR GROUND GRAB
                //ELEVATOR SHORT GROUND GRAB

                //All distances are in cm
                public static final double ELEVATOR_SHORT_GROUNDGRAB = 30.5;//12 in

                //ELEVATOR MEDIUM GROUND GRAB
                public static final double ELEVATOR_MEDIUM_GROUNDGRAB = 39.5;//15.5 in

                //ELEVATOR LONG GROUND GRAB
                public static final double ELEVATOR_LONG_GROUNDGRAB = 48; //18.9 in


                //public static final double ELEVATOR_CLIMB = 5; /* PENDING TEST */

                // BASKETS
                    /*
                    Clarification:
                     The following distances may seem to exceed the maximum length stipulated in rule R104, however,
                     the distances are with the arm at a high angle thus, the total horizontal expansion limit
                     is still acknowledged as per rule R104. This was calculated using trigonometry with the following formula.
                            HorizontalExtension = cosine(armAngle) * ElevatorExtension
                   */
                public static final double ELEVATOR_LOWBASKET = 30;//23 inches of extension
                public static final double ELEVATOR_HIGHBASKET = 61;//40 inches of extension

                // CHAMBERS
                public static final double ELEVATOR_LOWCHAMBER = 0; // 0 inches of extension
                public static final double ELEVATOR_HIGHCHAMBER = 25;//

            }
    /* ELEVATOR POSITIONS */
            public static class Wrist {
                // BASKETS
                public static final double WRIST_HIGHBASKET = 0;
                public static final double WRIST_LOWBASKET = 0;

                // GROUND GRAB
                public static final double WRIST_GROUNDGRAB_SHORT = 0.9;
                public static final double WRIST_GROUNDGRAB_MEDIUM = 0;
                public static final double WRIST_GROUNDGRAB_LONG = 0;

                // CHAMBERS
                public static final double WRIST_HIGHCHAMBER = 0;
                public static final double WRIST_LOWCHAMBER = 0;

                // TEST POSITIONS
                public static final double WRIST_SHORT = 0.3; //((Arm.ARM_SHORT_GROUNDGRAB + 90 + 11)/180)
                public static final double WRIST_MEDIUM = 0.5; //((Arm.ARM_MEDIUM_GROUNDGRAB + 90 + 11)/180);
                public static final double WRIST_LONG = 0.9; // (Arm.ARM_LONG_GROUNDGRAB + 90 + 11)/180);

            }
}
