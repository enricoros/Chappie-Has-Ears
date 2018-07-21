package com.enricoros.chappiehasears;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

public class XYInputView extends View {

    private final Paint mPtPaint;
    private final Paint mCtPaint;

    private long mStartTimeStamp = 0;

    private float mLastNx = 0;
    private float mLastNy = 0;

    public XYInputView(Context context, AttributeSet attrs) {
        super(context, attrs);

        mPtPaint = new Paint();
        mPtPaint.setColor(Color.GREEN);

        mCtPaint = new Paint();
        mCtPaint.setColor(Color.WHITE);
    }

    interface Listener {
        void onNewPoint(float nx, float ny);
    }
    private Listener mListener;
    public void setListener(Listener listener) {
        mListener = listener;
    }

    /**
     * @param nx In the -1 ... 1 range
     * @param ny In the -1 ... 1 range
     */
    public void setPoint(float nx, float ny) {
        if (mLastNx != nx || mLastNy != ny) {
            mLastNx = nx;
            mLastNy = ny;
            invalidate();
        }
    }


    @Override
    public boolean onTouchEvent(MotionEvent event) {
        final int W = getWidth();
        final int H = getHeight();

        mLastNx = normate(event.getX(), W);
        mLastNy = normate(event.getY(), H);
        if (mListener != null)
            mListener.onNewPoint(mLastNx, mLastNy);

        invalidate();
        return true;
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
