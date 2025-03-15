// SPDX-License-Identifier: CERN-OHL-P-2.0
// Copyright 2025 Jeff Lassahn
//
// Mechanical model of the GameSquirrel.

$fa=0.5;
$fs=0.4;

module Display()
{
    color("#808080")cube([60.26, 42.72, 2.25]);
    translate([8.63, 3.0, 0.2])color("#0000FF")cube([48.96, 36.72, 2.25]);

    color("#C04040")
    {
        translate([0,16.11,-1])cube([18,10.5,0.3]);
        translate([0,16.11, 0.5])rotate([-90,0,0])cylinder(h=10.5, r=1.5);
    }
}

module Joystick()
{
    color("#808080")
    {
        cube([13.15, 13.15, 11.7]);
        translate([13,1.825,0])cube([3,9.5,11.3]);
        translate([-1.025,1.825,0])cube([3,9.5,11.3]);
        translate([1.825,13,0])cube([9.5,3,11.3]);
        translate([1.825,-5,0])cube([9.5,6,6.5]);
    }
    color("#C04040")translate([6.575,6.575,0])cylinder(h=19.45, r=2);
}

module Button()
{
    color("#C04040")cylinder(h=9.5, r=1.75);
    color("#808080") translate([-3,-3.5,0]) cube([6,7,4]);
}

module PCB()
{
    color("#208020")cube([60,70,1.6]);
}

module Battery()
{
    color("#808080")cube([30.5, 42, 5.5]);
}

module SDCard()
{
    color("#808080")
    {
        cube([26.3, 16.5, 2.8]);
    }
    color("#C08080")
    {
        translate([1.15,4.3,0.35]) cube([24, 32, 2.1]);
    }
}

module USBPort()
{
    color("#808080")
    {
        translate([0,1.63,0]) cube([7.35, 8.94-3.26, 3.26]);
        translate([0,1.63,1.63]) rotate([0,90,0]) cylinder(h=7.35, r=1.63);
        translate([0,7.31,1.63]) rotate([0,90,0]) cylinder(h=7.35, r=1.63);
    }
}

module Switch()
{
    
    color("#808080")cube([3.6, 9.0, 3.5]);
    color("#C04040") translate([3.6,2.75,1]) cube([2,1.5,2]);
}

module AudioJack()
{
    color("#808080")
    {
        translate([0,2.5,0]) cube([6, 14.5, 5]);
        translate([3,0,2.5]) rotate([-90,0,0]) cylinder(h=2.5,r=2.5);
    }
}

module CaseBottom()
{
    difference()
    {
        cube([74,80, 15]);
        translate([2,2,2])cube([70,76,20]);
        translate([22.85, 76,8.7])cube([28.3, 5, 10]);
        translate([70,14.5,9.2]) cube([10,10,10]);

        translate([70,39.5,9.2]) cube([10,6,10]);
        translate([70,37.5,11.2]) cube([10,10,10]);
        translate([70,39.5,11.2]) rotate([0,90,0]) cylinder(h=10, r=2);
        translate([70,45.5,11.2]) rotate([0,90,0]) cylinder(h=10, r=2);
        
        translate([38,-1,12.2]) rotate([-90,0,0]) cylinder(h=10, r=3);
        translate([35,-1,12.2]) cube([6,10,10]);
        
        translate([37,26,-1]) cylinder(h=5, r=3.5); // FIXME hex nut
    }
    
    difference()
    {
        translate([31,20,0]) cube([12,12,8]);

        translate([37,26,-1]) cylinder(h=5, r=3.5); // FIXME hex nut
        translate([37,26,-1]) cylinder(h=10, r=1.8);
    }
    
    difference()
    {
        union()
        {
            translate([2,32,0]) cube([4,4,15]);
            translate([2,74.72,0]) cube([4,4,15]);
            translate([62.26,32,0]) cube([4,4,15]);
            translate([62.26,74.74,0]) cube([4,4,15]);
        }
        translate([3.75,33.75,14.2]) cube([60.76, 43.22, 2.5]);
    }
    
    translate([18,1,0]) cube([2,78,8]);
    translate([54,1,0]) cube([2,78,8]);
    translate([1,18,0]) cube([72,2,8]);
}

module CaseTop()
{
    difference()
    {
        translate([0,0,15]) cube([74,80, 2.45]);
        
        translate([11.63, 36.0, 0])cube([50.96, 38.72, 20]);
        translate([3.75,33.75,14.2]) cube([60.76, 43.22, 2.5]);
        translate([2.5,34.75,14.2]) cube([4, 40, 2.5]);

        translate([48,12,14]) cylinder(h=10, r=2.5);
        translate([48,26,14]) cylinder(h=10, r=2.5);
        translate([62,12,14]) cylinder(h=10, r=2.5);
        translate([62,26,14]) cylinder(h=10, r=2.5);
        
        translate([20.575, 18.575, 15]) cylinder(h=10, r=13.5, center=true);
        translate([37,26,17.5]) cylinder(h=-3, r1=3, r2=0);

        translate([37,26,14.5]) cylinder(h=3, r1=0, r2=3);        
        translate([37,26,10]) cylinder(h=10, r=1.5);        
    }

    translate([2.2,2.2,9.8]) cube([2,5,6]);
    translate([2.2,2.2,9.8]) cube([5,2,6]);
    translate([69.8,2.2,9.8]) cube([2,5,6]);
    translate([66.8,2.2,9.8]) cube([5,2,6]);
    translate([69.8,72.8,9.8]) cube([2,5,6]);
    translate([66.8,75.8,9.8]) cube([5,2,6]);    
}

module JoystickTop()
{
    difference()
    {
        union()
        {
            difference()
            {
                sphere(r=13);
                sphere(r=12);
                translate([-15,-15,-15])cube([30,30,15]);
            }
            translate([0,0,7])cylinder(h=10, r=4);
            intersection()
            {
                translate([0,0,15])sphere(r=8);
                translate([0,0,15])cube([20,20,8], center=true);
            }
        }
        
        intersection()
        {
            translate([0,0,-1])cylinder(h=15, d=4.1);
            cube([3.1,5,30], center=true);
        }
    }

}

module Screw()
{
    //M3x16mm
    color("#404040")
    {
        cylinder(h=14.4, r=1.5);
        translate([0,0,14.35]) cylinder(h=1.65, r1=1.5, r2=2.8);
    }
}

module Nut()
{
    translate([0,0,1.2]) color("#404040") intersection()
    {
        rotate([0,0,0]) cube([5.5, 10, 2.4], center=true);
        rotate([0,0,60]) cube([5.5, 10, 2.4], center=true);
        rotate([0,0,120]) cube([5.5, 10, 2.4], center=true);
    }
}

union()
{
    translate([7,4,8]) PCB();
    translate([14, 12, 9.7]) Joystick();
    translate([4, 34, 14.2]) Display();
    translate([48, 12, 9.7]) Button();
    translate([48, 26, 9.7]) Button();
    translate([62, 12, 9.7]) Button();
    translate([62, 26, 9.7]) Button();
    translate([23.85, 61.5, 9.7]) SDCard();
    translate([72-7.35,38,9.7]) USBPort();
    translate([69.5,15,9.7]) Switch();
    translate([21.75, 34, 2.5]) Battery();
    translate([35,0,9.7]) AudioJack();

    translate([37,26,1.5])
    {
        Screw();
        Nut();
    }
}

intersection()
{
    CaseBottom();
    //cube([36,85,20]);
}
CaseTop();

// Joystick rotation point is 6.45 above seating plane, 30 degree travel
translate([14 + 6.575, 12 + 6.575, 9.7 + 6.45]) JoystickTop();
