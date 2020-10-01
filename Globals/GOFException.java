package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals;

public class GOFException extends RuntimeException {

    private String text;

    /**
     * Constructor to build throwable exception
     * @param err Text to display with exception
     */
    public GOFException(String err) {
        text = err;
    }

    /**
     * Generate Exception text
     * @return String of GOFException text
     */
    public String toString() {
        return "516 Gears of Fire Exception: " + text + ".  Aren't you glad to be on a team that has *custom error messages* to entertain you as your code fails miserably :D :D :D?";
    }
}