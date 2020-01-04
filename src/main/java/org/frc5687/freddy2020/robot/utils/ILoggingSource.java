package org.frc5687.freddy2020.robot.utils;

public interface ILoggingSource {
    void error(String message);
    void warn(String message);
    void info(String message);
    void debug(String message);
}
