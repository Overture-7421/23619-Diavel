package org.firstinspires.ftc.teamcode.Subsystems;

public final class Constants {
    private Constants(){}

    public static class Arm {
        /* ARM POSITIONS */
            public static final double ARM_STOW = -33;

            // GROUND GRAB
                // SHORT GROUND GRAB
                public static final double ARM_SHORT_GROUNDGRAB = -37;

                // MEDIUM GROUND GRAB
                public static final double ARM_MEDIUM_GROUNDGRAB = -32.5;

                // LONG GROUND GRAB
                public static final double ARM_LONG_GROUNDGRAB = -28;



            public static final double ARM_CLIMB = -16; /* PENDING TEST */

            // BASKETS
            public static final double ARM_LOWBASKET = 40;
            public static final double ARM_HIGHBASKET = 55;

            // CHAMBERS
            public static final double ARM_LOWCHAMBER = 0;
            public static final double ARM_HIGHCHAMBER = 24;// 15 si se hace de abajo-arriba y 90 si se hace de arriba-abajo
    }

    public static class Elevator {
        /* ELEVATOR POSITIONS */
            public static final double ELEVATOR_STOW = 0;

            //ELEVATOR GROUND GRAB
                //ELEVATOR SHORT GROUND GRAB

                //All distances are in cm
                public static final double ELEVATOR_SHORT_GROUNDGRAB = 30.5;//12 in

                //ELEVATOR MEDIUM GROUND GRAB
                public static final double ELEVATOR_MEDIUM_GROUNDGRAB =  39.5;//15.5 in

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
            public static final double ELEVATOR_HIGHBASKET = 69;//40 inches of extension

            // CHAMBERS
            public static final double ELEVATOR_LOWCHAMBER = 0; // 0 inches of extension
            public static final double ELEVATOR_HIGHCHAMBER = 30;//26.2 inches of extension
    }
}
