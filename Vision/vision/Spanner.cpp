//FIXME - On diamond-perimeter-shaped regions, counts the top multiple times.
//  This blows up badly on fine grids.

#include "Spanner.h"
#include "Colorseg.h"
#include "Transform.h"
#include "Distortion.h"

#include "../Config_File.h"

#include <QMutexLocker>
#include <boost/foreach.hpp>

using namespace std;

Vision::Group::Group()
{
    num_pixels = 0;
    good_pixels = 0;
    id = -1;

    min_x = max_x = -1;
    min_y = max_y = -1;
}

////////

Vision::Spanner::Spanner(Colorseg *colorseg, int color)
{
    _colorseg = colorseg;
    _color = color;

    distortion = 0;

    max_gap = 0;
    min_span_pixels = 3;
    min_group_width = 3;
    max_group_width = 20;
    min_group_height = 3;
    max_group_height = 20;
    min_group_aspect = 0.5;
    max_group_aspect = 2.0;

    debug_color = -1;
}

void Vision::Spanner::run()
{
    QMutexLocker ml(&mutex);

    const QImage &cs_output = _colorseg->output();
    _width = cs_output.width();
    _height = cs_output.height();

    // Find spans on each row independently
    find_spans();

    // Find adjacencies
    for (int y = 0; y < (_height - 1); ++y)
    {
        BOOST_FOREACH(Span *span, _spans[y])
        {
            BOOST_FOREACH(Span *other, _spans[y + 1])
            {
                if (span->overlaps(other))
                {
                    span->adjacencies.push_back(other);
                    other->adjacencies.push_back(span);
                }
            }
        }
    }

    // Delete all groups
    BOOST_FOREACH(Group *group, _groups)
    {
        delete group;
    }
    _groups.clear();

    // Make new groups
    BOOST_FOREACH(Span *span, _all_spans)
    {
        if (span->group)
        {
            continue;
        }

        Group *group = new Group();
        group->color = _color;

        // Propagate the group and accumulate centers
        span->group = group;
        propagate_group(span);

        // Reject this group if it's the wrong size or shape
        int w = group->width();
        int h = group->height();
        float aspect = (float)w / (float)h;
        if (_color == debug_color)
        {
            printf("group %p (%d, %d)-(%d, %d)\n", group, group->min_x, group->min_y, group->max_x, group->max_y);
        }

        if (w < min_group_width || w > max_group_width ||
            h < min_group_height || h > max_group_height ||
            aspect < min_group_aspect || aspect > max_group_aspect)
        {
            if (_color == debug_color)
            {
                printf("reject group %p: %d, %d, %.2f\n", group, w, h, aspect);
            }

            delete group;
            continue;
        }

        // Calculate average center
        group->raw_center.x /= (float)group->num_pixels;
        group->raw_center.y /= (float)group->num_pixels;

        // Undistort and transform to world space
    	group->center = transform->transform(distortion->undistort(group->raw_center));

        // Add this group to the list
        _groups.push_back(group);
    }
}

void Vision::Spanner::find_spans()
{
    if (_color == debug_color)
    {
        printf("\n");
    }

    // Make sure we have the right number of span lists
    if (_height != (int)_spans.size())
    {
        _spans.resize(_height);
    }

    // Clear the list of all spans (shouldn't be any left, though).
    // The spans will be deleted below.
    _all_spans.clear();

    for (int y = 0; y < _height; ++y)
    {
        // Delete existing spans on this line
        BOOST_FOREACH(Span *span, _spans[y])
        {
            delete span;
        }
        _spans[y].clear();

        // Index of the first pixel on this line
        int start = _colorseg->line_start[_color][y];

        // Index one after the last pixel on this line
        int end = _colorseg->line_start[_color][y + 1];

        // Skip if there are no pixels on this line
        if ((end - start) < min_span_pixels)
        {
            continue;
        }

#if 0
        if (debug)
        {
            printf("%d: %d-%d %d-%d\n", y, start, end, cs->color_list[color][start], cs->color_list[color][end]);
            for (int j = start; j < end; ++j)
                printf(" %d", cs->color_list[color][j] - y * width);
            printf("\n");
        }
#endif

        // Index of the first pixel in the first span
        int span_start = start;

        // X of the first pixel in the span
        int start_x = _colorseg->color_list[_color][start] - y * _width;

        // X of the last pixel checked
        int last_x = start_x;

        //FIXME - Can the last span on a line be wrong?

        // The span starts on (start).  Check the next pixels up to the end of the line.
        // i is the index of the pixel just past the end of the span.
        for (int i = start + 1; i <= end; ++i)
        {
            // X of this pixel
            int x = _colorseg->color_list[_color][i] - y * _width;

            // Distance between this pixel and the last one of the color
            // (subtract one so two adjacent pixels have a gap of zero)
            int gap = x - last_x - 1;

            if (gap > max_gap || i == end)
            {
                // End the span on the previous pixel
                int good_pixels = i - span_start;

                if (good_pixels >= min_span_pixels)
                {
                    Span *s = new Span(start_x, last_x, y, good_pixels);
                    _spans[y].push_back(s);
                    _all_spans.push_back(s);

                    if (_color == debug_color)
                    {
                        int distance = last_x - start_x + 1;
                        printf("span on %d: %3d-%3d good %3d dist %3d\n",
                            y, start_x, last_x, good_pixels, distance);
                    }
                }

                // The next span starts here
                start_x = x;
                span_start = i;
            }

            last_x = x;
        }
    }
}

void Vision::Spanner::propagate_group(Span *span)
{
    Group *group = span->group;

    // Number of pixels covered by the span
    int n = span->x2 - span->x1 + 1;

    // Add to average center
    group->raw_center.x += (span->x1 + span->x2) / 2.0 * n;
    group->raw_center.y += span->y * n;

    // Collect statistics
    group->num_pixels += n;
    group->good_pixels += span->good_pixels;

    if (group->min_x < 0 || span->x1 < group->min_x)
    {
        group->min_x = span->x1;
    }

    if (group->max_x < 0 || span->x2 > group->max_x)
    {
        group->max_x = span->x2;
    }

    if (group->min_y < 0 || span->y < group->min_y)
    {
        group->min_y = span->y;
    }

    if (group->max_y < 0 || span->y > group->max_y)
    {
        group->max_y = span->y;
    }

    BOOST_FOREACH(Span *other, span->adjacencies)
    {
        if (!other->group)
        {
            other->group = group;
            propagate_group(other);
        }
    }
}

void Vision::Spanner::load(QDomElement element)
{
	read_param(element, "max_gap", max_gap);
	read_param(element, "min_span_pixels", min_span_pixels);
	read_param(element, "min_group_width", min_group_width);
	read_param(element, "max_group_width", max_group_width);
	read_param(element, "min_group_height", min_group_height);
	read_param(element, "max_group_height", max_group_height);
	read_param(element, "min_group_aspect", min_group_aspect);
	read_param(element, "max_group_aspect", max_group_aspect);
}

void Vision::Spanner::save(QDomElement element)
{
	element.setAttribute("color", color());
	element.setAttribute("max_gap", max_gap);
	element.setAttribute("min_span_pixels", min_span_pixels);
	element.setAttribute("min_group_width", min_group_width);
	element.setAttribute("max_group_width", max_group_width);
	element.setAttribute("min_group_height", min_group_height);
	element.setAttribute("max_group_height", max_group_height);
	element.setAttribute("min_group_aspect", min_group_aspect);
	element.setAttribute("max_group_aspect", max_group_aspect);
}
