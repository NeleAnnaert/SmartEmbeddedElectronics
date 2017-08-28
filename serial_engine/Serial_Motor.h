typedef struct command{
    // m: Move, r: Rotate
    char type,
    // m: f(orward),b(ackward) r: l(eft),r(ight)
    char direction,
    // Speed defined between 0 and 200 (Speeds above 200 should only be used for short periods)
    int speed,
    // Rotating angle defined between 0° and 180°
    int angle,
    // Distance in mm
    int distance;
};
