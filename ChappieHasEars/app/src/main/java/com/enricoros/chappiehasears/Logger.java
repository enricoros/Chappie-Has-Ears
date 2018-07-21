package com.enricoros.chappiehasears;

import android.content.Context;
import android.util.Log;
import android.widget.Toast;

public class Logger {
	private static final boolean ENABLED = true;
	private static final String LOGTAG = "ChappieHasEars";


	public static void notImplemented(String msg) {
		error("notImplemented(): "+msg);
	}

    public static void apiUnreachable(String msg) {
        error("unreachable(): "+msg);
    }

	public static void debug(String msg) {
		if (ENABLED)
			Log.i(LOGTAG, msg);
	}

    public static void info(String msg) {
        if (ENABLED)
            Log.i(LOGTAG, msg);
    }

	public static void warn(String msg) {
		if (ENABLED)
			Log.w(LOGTAG, msg);
	}

	public static void error(String msg) {
		if (ENABLED)
			Log.e(LOGTAG, msg);
	}

	public static void wtf(String msg) {
		error(msg);
	}

	public static void apierror(String msg) {
		error(msg);
	}

	public static void exception(String msg, Exception e) {
		error(msg+", because: "+e.getMessage()+". Stack trace:");
		if (ENABLED)
			e.printStackTrace();
	}

    public static void userVisibleMessage(String msg) {
        info("Toast: '"+msg+"'");
        if (mContext != null)
            Toast.makeText(mContext, msg, Toast.LENGTH_SHORT).show();
    }

    public static void userVisibleException(Exception e, String msg) {
        String toast = msg + " Exception: " + e.getMessage();
        info("Toast: '"+toast+"'");
        if (mContext != null)
            Toast.makeText(mContext, toast, Toast.LENGTH_LONG).show();
    }

    public static class TimeKeeper {
        private long mTotalTime = 0;
        private long mTotalIntervals = 0;
        boolean mStarted = false;
        private long mStartTime = 0;
        private long mStopTime = 0;
        private long mLastGap;

        public void start() {
            if (mStarted)
                Logger.warn("Calling TimeKeeper.start while not stopped");
            mStarted = true;
            mStartTime = System.currentTimeMillis();
        }

        public int stop() {
            if (!mStarted) {
                Logger.warn("Calling TimeKeeper.stop while not started");
                return 0;
            }
            mStarted = false;
            mStopTime = System.currentTimeMillis();
            mLastGap = mStopTime - mStartTime;
            mTotalIntervals++;
            mTotalTime += mLastGap;
            return (int) mLastGap;
        }

        public void printStats(String prefix) {
            Logger.wtf(prefix+": "+mLastGap+" ms, average: "+average()+" ms.");
        }

        public float average() {
            return mTotalIntervals > 0 ? (float)mTotalTime / (float)mTotalIntervals : 0;
        }

    }

    private static Context mContext = null;
    public static void setContextForUIMessages(Context context) {
        mContext = context;
    }

}
