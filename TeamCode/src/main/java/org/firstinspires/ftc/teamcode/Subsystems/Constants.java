package org.firstinspires.ftc.teamcode.Subsystems;

public final class Constants {
    private Constants(){}

    public static class Arm {
        /* ARM POSITIONS */
            public static final double ARM_STOW = -33;

            // GROUND GRAB
                // SHORT GROUND GRAB
                public static final double ARM_SHORT_GROUNDGRAB = -34;

                // MEDIUM GROUND GRAB
                public static final double ARM_MEDIUM_GROUNDGRAB = -29;

                // LONG GROUND GRAB
                public static final double ARM_LONG_GROUNDGRAB = -24;



            public static final double ARM_CLIMB = 75; /* PENDING TEST */

            // BASKETS
            public static final double ARM_LOWBASKET = 60;
            public static final double ARM_HIGHBASKET = 60;

            // CHAMBERS
            public static final double ARM_LOWCHAMBER = 0;
            public static final double ARM_HIGHCHAMBER = 38;// 15 si se hace de abajo-arriba y 90 si se hace de arriba-abajo
    }

    public static class Elevator {
        /* ELEVATOR POSITIONS */
            public static final double ELEVATOR_STOW = 0;

            //ELEVATOR GROUND GRAB
                //ELEVATOR SHORT GROUND GRAB
                public static final double ELEVATOR_SHORT_GROUNDGRAB = 30.5;

                //ELEVATOR MEDIUM GROUND GRAB
                public static final double ELEVATOR_MEDIUM_GROUNDGRAB =  39.5;

                //ELEVATOR LONG GROUND GRAB
                public static final double ELEVATOR_LONG_GROUNDGRAB = 46.5;


            public static final double ELEVATOR_CLIMB = 5; /* PENDING TEST */

            // BASKETS
            public static final double ELEVATOR_LOWBASKET = 15;
            public static final double ELEVATOR_HIGHBASKET = 69;

            // CHAMBERS
            public static final double ELEVATOR_LOWCHAMBER = 0;
            public static final double ELEVATOR_HIGHCHAMBER = 5;//0 si se hace de arriba-abajo y 5 si se hace de abajo-arriba

    }
}
