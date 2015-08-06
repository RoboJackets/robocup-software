#pragma once

#include <QtWidgets>

#include "Field_Dimensions.hpp"
#include <Geometry2d/TransformMatrix.hpp>


/// Orientation of the drawn field, with our goal as the reference point.
/// Each orientation is 90-degrees counter-clockwise from the previous.
typedef enum {
    FieldOrientationLandscapeLeft = 0,
    FieldOrientationPortrait,
    FieldOrientationLandscapeRight,
    FieldOrientationPortraitUpsideDown
} FieldOrientation;

inline bool FieldOrientationIsPortrait(FieldOrientation orientation) {
    return orientation == FieldOrientationPortraitUpsideDown ||
           orientation == FieldOrientationPortrait;
}

inline FieldOrientation FieldOrientationRotate180(
    FieldOrientation orientation) {
    static FieldOrientation flippedOrientations[] = {
        FieldOrientationLandscapeRight,
        FieldOrientationPortraitUpsideDown,
        FieldOrientationLandscapeLeft,
        FieldOrientationPortrait
    };
    return flippedOrientations[orientation];
}

/// This class draws the field including lines, goals, goal zones, etc.
/// It also handles coordinate conversions related to drawing.
class FieldBackgroundView : public QWidget {
public:
    FieldBackgroundView(QWidget* parent = nullptr);

    const Field_Dimensions& fieldDimensions() const {
        return _fieldDimensions;
    }
    void setFieldDimensions(const Field_Dimensions& fieldDimensions) {
        _fieldDimensions = fieldDimensions;
    }

    /// Set which team we are (for the team transformations).
    /// Defaults to false (us as the yellow team).
    void setBlueTeam(bool blue) {
        _blueTeam = blue;
        invalidateTransforms();
        update();
    }
    bool isBlueTeam() const {
        return _blueTeam;
    }

    /// Which end of the field we're defending (relative to global coordinates).
    /// Defaults to true
    void setDefendingPlusX(bool defendingPlusX) {
        _defendingPlusX = defendingPlusX;
        invalidateTransforms();
        update();
    }
    bool defendingPlusX() const {
        return _defendingPlusX;
    }

    /// Transforms the painter's coordinates so that (0, 0) is the field center,
    /// drawing units correspond to meters, and the field rotation is applied.
    void setupWorldSpace(QPainter* painter) const;

    /// Field orientation - relative to the world coordinate
    void setFieldOrientation(FieldOrientation orientation);
    FieldOrientation fieldOrientation() const {
        return _fieldOrientation;
    }

    /// Get geometry transformations.  Ensure they're valid by calling
    /// recalculateTransformsIfNeeded() first.
    const Geometry2d::TransformMatrix& screenToWorld() const {
        return _screenToWorld;
    }
    const Geometry2d::TransformMatrix& worldToTeam() const {
        return _worldToTeam;
    }
    const Geometry2d::TransformMatrix& teamToWorld() const {
        return _teamToWorld;
    }
    int textRotationForWorldSpace() const {
        return _textRotationWorldSpace;
    }
    int textRotationForTeamSpace() const {
        return _textRotationTeamSpace;
    }

    /// Recalculates all transforms if @_transformsValid is false.
    void recalculateTransformsIfNeeded() const;


protected:
    virtual void paintEvent(QPaintEvent* e);

    /// Override resizeEvent() so we can enforce proper x-y aspect ratio
    virtual void resizeEvent(QResizeEvent* e);

    /// Set transforms to be recalculated.
    void invalidateTransforms() {
        _transformsValid = false;
    }

private:
    Field_Dimensions _fieldDimensions;

    bool _blueTeam;
    bool _defendingPlusX;

    mutable bool _transformsValid;

    /// Coordinate transformations
    mutable Geometry2d::TransformMatrix _screenToWorld;
    mutable Geometry2d::TransformMatrix _worldToTeam;
    mutable Geometry2d::TransformMatrix _teamToWorld;

    // How many degrees to rotate text (in world space) so it draws upright on
    // screen.
    mutable int _textRotationWorldSpace;
    mutable int _textRotationTeamSpace;

    /// Defaults to FieldOrientationPortrait
    FieldOrientation _fieldOrientation;
};
