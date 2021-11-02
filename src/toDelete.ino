/*
*
*   EXAMPLE FINITE STATE MACHINE
*
*/
typedef enum
{
    IDLE = 1,
    PAGE_TURN_LEFT = 2,
    PAGE_TURN_RIGHT = 3,
    SWING = 4,
    // .. continue ...
} state;
state CURRENT_STATE;
bool turnPageLeftFlag = false;
bool turnPageRightFlag = false;
void setup()
{
}
void loop()
{
    update_state(millis());
}
void update_state(long mils)
{
    state NEXT_STATE = CURRENT_STATE;
    switch (CURRENT_STATE)
    {
        case IDLE:
            // code
            NEXT_STATE = state_idle();
            break;
        case PAGE_TURN_LEFT:
            // Perform routine to turn page left
            break;
        case PAGE_TURN_RIGHT:
            // Perform routine to turn page right
            break;
        default:
            break;
    }
    CURRENT_STATE = NEXT_STATE;
}
state state_idle() {
    if(turnPageLeftFlag) {
        turnPageLeftFlag = false;
        return PAGE_TURN_LEFT;
    }
    else if(turnPageLeftFlag) {
        turnPageRightFlag = false;
        return PAGE_TURN_RIGHT;
    }
    else {
        return IDLE;
    }
}
void page_turn_left_interrupt()
{
    turnPageLeftFlag = false;
}
void page_turn_right_interrupt()
{
    turnPageRightFlag = true;
}