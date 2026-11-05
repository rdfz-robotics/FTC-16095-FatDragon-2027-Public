package android.util;

/** Host-only logging bridge for route tests; never included in the Android application. */
public final class Log {
    private Log() { }

    public static int println(int priority, String tag, String message) {
        System.out.println(tag + ": " + message);
        return message == null ? 0 : message.length();
    }
}
