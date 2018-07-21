package com.enricoros.chappiehasears;

import android.annotation.SuppressLint;
import android.content.Context;
import android.os.Looper;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.widget.TextView;
import android.widget.Toast;

import java.util.Calendar;

@SuppressLint("SetTextI18n")
public class Logger {
    private static final boolean ENABLED = true;
    private static final String LOGTAG = "ChappieHasEars";

    public static void notImplemented(String msg) {
        error("notImplemented(): " + msg);
    }

    public static void apiUnreachable(String msg) {
        error("unreachable(): " + msg);
    }

    public static void debug(String msg) {
        if (ENABLED)
            Log.i(LOGTAG, msg);
        logView("D: " + msg);
    }

    public static void info(String msg) {
        if (ENABLED)
            Log.i(LOGTAG, msg);
        logView("I: " + msg);
    }

    public static void warn(String msg) {
        if (ENABLED)
            Log.w(LOGTAG, msg);
        logView("W: " + msg);
    }

    public static void error(String msg) {
        if (ENABLED)
            Log.e(LOGTAG, msg);
        logView("E: " + msg);
    }

    public static void wtf(String msg) {
        error(msg);
    }

    public static void apierror(String msg) {
        error(msg);
    }

    public static void exception(String msg, Exception e) {
        error(msg + ", because: " + e.getMessage() + ". Stack trace:");
        if (ENABLED)
            e.printStackTrace();
    }

    public static void userVisibleMessage(String msg) {
        info("Toast: '" + msg + "'");
        if (mContext != null)
            Toast.makeText(mContext, msg, Toast.LENGTH_SHORT).show();
    }

    public static void userVisibleException(Exception e, String msg) {
        String toast = msg + " Exception: " + e.getMessage();
        info("Toast: '" + toast + "'");
        if (mContext != null)
            Toast.makeText(mContext, toast, Toast.LENGTH_LONG).show();
    }

    private static Context mContext = null;
    private static TextView mLogView = null;

    static void setContextForUIMessages(Context context, TextView logView) {
        mContext = context;
        mLogView = logView;
        if (mLogView != null)
            mLogView.setMovementMethod(new ScrollingMovementMethod());
    }

    private static boolean onUiThread() {
        return mLogView != null && Looper.myLooper() == Looper.getMainLooper();
    }

    private static void logView(String message) {
        if (!onUiThread())
            return;
        final Calendar calendar = Calendar.getInstance();
        message = calendar.get(Calendar.HOUR_OF_DAY) + ":" + calendar.get(Calendar.MINUTE) + ":" + calendar.get(Calendar.SECOND) + " " + message;
        mLogView.setText(message + "\n" + mLogView.getText());
    }

}
