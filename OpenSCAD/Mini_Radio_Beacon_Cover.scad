// Simple Cover for the Iridium Mini Radio Beacon with USB opening

$fn=90; // fragments
wall = 2.0; // wall thickness
end_wall = 3.0; // end wall thickness
internal_width = 56.5; // internal width (X) of the cover
internal_depth = 30.0; // internal depth (Y) of the cover
internal_height = 35.0; // internal height (Z) of the cover
lid_insert = 1.0; // extra height for the lid insert

battery_support_height = 1.0; // Height (Z) of the battery support above the floor
battery_support_depth = 2.0; // Depth (Y) of the battery support
battery_support_offset = 5.0; // Y offset of the battery support

battery_support_2_width = 1.0; // Width (X) of the battery support from the wall
battery_support_2_height = 27.0; // Internal height (Z) of the battery support

internal_corner_radius = 1.0; // radius of the internal corner cylinder

usb_r = 2.0; // radius of the opening for the USB connector
usb_width = 10.0; // total width of the opening for the USB
usb_height = 1.0; // thickness of the opening for the USB connector
usb_recess_r = 4.0; // radius of the recess for the USB connector body
usb_recess_width = 12.0; // total width of the recess for the USB connector body
usb_x_offset = -13.8; // X offset of the USB connector
usb_y_offset = 5.75; // y offset of the USB connector

height = internal_height + end_wall + lid_insert; // cover external height (Z)
width = internal_width + (2 * wall); // cover external width (X)
depth = internal_depth + (2 * wall); // cover external depth (Y)

external_corner_radius = internal_corner_radius + wall; // external corner radius

module cover()
{
    translate([external_corner_radius, external_corner_radius, 0])
        minkowski() {
            cube([(width - (2 * external_corner_radius)), (depth - (2 * external_corner_radius)), (height / 2)]);
            cylinder(h=(height / 2), r=external_corner_radius);
        }
}

module void()
// Void will be higher than required to avoid zero thickness skin across opening
{
    translate([external_corner_radius, external_corner_radius, end_wall])
        minkowski() {
            cube([(internal_width - (2 * internal_corner_radius)), (internal_depth - (2 * internal_corner_radius)), (height / 2)]);
            cylinder(h=(height / 2), r=internal_corner_radius);
        }
}

module usb_opening_end_1()
// Cylinder is taller than it needs to be to avoid zero thickness skins
{
    translate([((usb_width / 2) - usb_r),0,usb_height]) {
        cylinder(h=end_wall,r=usb_r);
    }
}

module usb_opening_end_2()
// Cylinder is taller than it needs to be to avoid zero thickness skins
{
    translate([(0-((usb_width / 2) - usb_r)),0,usb_height]) {
        cylinder(h=end_wall,r=usb_r);
    }
}

module usb_opening_mid()
// Cube is taller than it needs to be to avoid zero thickness skins
{
    translate([(-0.5 * (usb_width - (2 * usb_r))), (-0.5 * (2 * usb_r)), usb_height]) {
        cube([(usb_width - (2 * usb_r)), (2 * usb_r), end_wall]);
    }
}

module usb_recess_end_1()
// Cylinder is taller than it needs to be to avoid zero thickness skins
{
    translate([((usb_recess_width / 2) - usb_recess_r), 0, (0 - usb_height)]) {
        cylinder(h=end_wall, r=usb_recess_r);
    }
}

module usb_recess_end_2()
// Cylinder is taller than it needs to be to avoid zero thickness skins
{
    translate([(0-((usb_recess_width / 2) - usb_recess_r)), 0, (0 - usb_height)]) {
        cylinder(h=end_wall, r=usb_recess_r);
    }
}

module usb_recess_mid()
// Cube is taller than it needs to be to avoid zero thickness skins
{
    translate([(-0.5 * (usb_recess_width - (2 * usb_recess_r))), (-0.5 * (2 * usb_recess_r)), (0 - usb_height)]) {
        cube([(usb_recess_width - (2 * usb_recess_r)), (2 * usb_recess_r), end_wall]);
    }
}

module usb()
{
    translate([((width / 2) + usb_x_offset), ((depth / 2) - usb_y_offset), 0]) {
        union() {
            usb_opening_end_1();
            usb_opening_end_2();
            usb_opening_mid();
            usb_recess_end_1();
            usb_recess_end_2();
            usb_recess_mid();
        }
    }
}

module battery_support()
{
    translate([0, ((depth/2) + battery_support_offset), 0]) {
        cube([width, battery_support_depth, (end_wall + battery_support_height)]);
    }
}

module battery_support_2()
{
    translate([(width - (wall + battery_support_2_width)), ((depth/2) + battery_support_offset), 0]) {
        cube([(wall + battery_support_2_width), battery_support_depth, (end_wall + battery_support_2_height)]);
    }
}

module finished_cover()
{
    union() {
        difference() {
            cover();
            void();
            usb();
        }
        battery_support();
        battery_support_2();
    }
}

finished_cover();

