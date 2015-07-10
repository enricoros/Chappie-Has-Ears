package com.enricoros.chappiehasears;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.os.SystemClock;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

public class XYInputView extends View {

    private final Paint mPtPaint;
    private final Paint mCtPaint;

    private long mStartTimeStamp = 0;

    private float mLastNx = 0;
    private float mLastNy = 0;
    private long mLastDTimeMs;

    static interface Listener {

        void onNewPoint(float nx, float ny, long timeStampMs);

    }
    private Listener mListener;

    public XYInputView(Context context, AttributeSet attrs) {
        super(context, attrs);

        mPtPaint = new Paint();
        mPtPaint.setColor(Color.GREEN);

        mCtPaint = new Paint();
        mCtPaint.setColor(Color.WHITE);
    }

    public void setListener(Listener listener) {
        mListener = listener;
    }

    /**
     * @return The reference time of start
     */
    long startRecording() {
        mStartTimeStamp = SystemClock.uptimeMillis();
        mPtPaint.setColor(Color.RED);
        invalidate();
        return mStartTimeStamp;
    }

    void stopRecording() {
        mStartTimeStamp = 0;
        mPtPaint.setColor(Color.GREEN);
        invalidate();
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        // if not started, just ignore the event
        //if (mStartTimeStamp == 0)
        //    return super.onTouchEvent(event);

        final int W = getWidth();
        final int H = getHeight();
        mLastDTimeMs = SystemClock.uptimeMillis() - mStartTimeStamp;
        mLastNx = normate(event.getX(), W);
        mLastNy = normate(event.getY(), H);

        /*int hCount = event.getHistorySize();
        for (int i = 0; i < hCount; ++i) {
            final long hDTimeMs = event.getHistoricalEventTime(i) - mStartTimeStamp;
            final float hNX = normate(event.getHistoricalX(i), W);
            final float hNY = normate(event.getHistoricalY(i), H);
            transmit(hNX, hNY, hDTimeMs);
            //Logger.wtf("TraceRecordView.historical: "+i+" d "+hDTimeMs);
        }*/

        transmit(mLastNx, mLastNy, mLastDTimeMs);

        invalidate();
        return true;
    }

    // delayed transmission for a 10 Hz refresh rate max
    float mNextNx = 0;
    float mNextNy = 0;
    long mNextdTimeMs = 0;
    private boolean mNextTransmissionPending = false;
    private Runnable mTransmissionRunnable = new Runnable() {
        @Override
        public void run() {
            if (mListener != null)
                mListener.onNewPoint(mNextNx, mNextNy, mNextdTimeMs);
            mNextTransmissionPending = false;
        }
    };

    private void transmit(float nx, float ny, long dTimeMs) {
        final long timeMillis = System.currentTimeMillis();
        mNextNx = nx;
        mNextNy = ny;
        mNextdTimeMs = dTimeMs > 0 ? dTimeMs : 0;
        if (!mNextTransmissionPending) {
            postDelayed(mTransmissionRunnable, 40);
            mNextTransmissionPending = true;
        }
    }


    @Override
    protected void onDraw(Canvas canvas) {
        final int W = getWidth();
        final int H = getHeight();

        canvas.drawCircle(expand(mLastNx, W), expand(mLastNy, H), 24, mPtPaint);

        canvas.drawCircle(expand(0, W), expand(0, H), 8, mCtPaint);
    }

    private float normate(float expandedVal, float scale) {
        float v = 2.0f * expandedVal / scale - 1;
        return v > -1 ? (v < 1 ? v : 1) : -1;
    }

    private int expand(float normVal, float scale) {
        return Math.round((normVal + 1) / 2f * scale);
    }

}
