/**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */

 package frc.lib.util;

 public class LimelightOffsets {
     
     public double driveOffset;
     public double strafeOffset;
     public double rotationOffset;
 
     public LimelightOffsets(
        double driveValue,
        double strafeValue,
        double rotationOffset
         
     ){
         this.driveOffset = driveValue;
         this.strafeOffset = strafeValue;
         this.rotationOffset = rotationOffset;
         
 
     }
 }
 