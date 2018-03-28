import java.io.*;
import java.nio.file.Files;
import java.security.DigestInputStream;
import java.security.MessageDigest;
import java.util.Scanner;


public class FileInput {

	public String getMD5(File file) throws Exception {
		 MessageDigest md = MessageDigest.getInstance("MD5");
		 InputStream is = Files.newInputStream(file.toPath());
		 DigestInputStream dis = new DigestInputStream(is, md); 

		 byte[] digest = md.digest();
		 return digest.toString();
	}

	public static void serializeSplineTraj(Trajectory traj, String filename) {
		try {
			System.out.println("Begin Serialization of Trajectory");
			File realFile = new File(filename);
			System.out.println("Creating New File: " + realFile.getAbsolutePath());
			realFile.createNewFile();

			FileOutputStream fos = new FileOutputStream(realFile); 
			BufferedWriter bw = new BufferedWriter(new OutputStreamWriter(fos)); 
			
			for(int i=0; i<traj.getNumSegments(); i++) {
				Trajectory.Segment seg = traj.getSegment(i);
				bw.write(seg.pos + " " + seg.vel + " " + seg.acc + " " + seg.dt + " " + seg.heading + " " + seg.jerk + " " + seg.x + " " + seg.y);
				bw.write('\n');
			}
			
			bw.close();
		} catch(Exception e) {
			e.printStackTrace();
		}
        System.out.println("Trajectory has been serialized!");
	}
}
