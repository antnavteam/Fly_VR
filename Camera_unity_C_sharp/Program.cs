using System.Runtime.InteropServices;
using System;
using System.Drawing; // For image handling
using System.IO;

class Program
{
    public struct Struct_result
    {
        public double headingAngle;
        public double frame_rate;
        public int wings;
        //public IntPtr display_image; // Change to IntPtr
        public IntPtr display_image;  // Pointer to the image data (byte array)
        public int display_image_size; // Size of the image data
    };

    // Define the callback function signature
    public delegate void CallbackFunction(Struct_result struct_result);

    //[DllImport("camera_acquisition_standard.dll", CallingConvention = CallingConvention.Cdecl)]
    [DllImport("camera_acquisition_debug_print_img.dll", CallingConvention = CallingConvention.Cdecl)]
    //[DllImport("camera_acquisition_no_img.dll", CallingConvention = CallingConvention.Cdecl)]
    public static extern int AcquireImages(CallbackFunction callback);

    // Callback implementation
    static void ProgressCallback(Struct_result struct_result)
    {

        // Allocate a byte array of the appropriate size
        byte[] displayImage = new byte[struct_result.display_image_size];

        // Marshal the unmanaged memory (IntPtr) to the managed byte array
        Marshal.Copy(struct_result.display_image, displayImage, 0, struct_result.display_image_size);


        Console.WriteLine($"Callback received - frame_rate: {struct_result.frame_rate}, Heading: {struct_result.headingAngle}");
    }

    static void Main()
    {
        Console.WriteLine("Starting function...");

        // Call the DLL function and provide the callback
        try
        {
            AcquireImages(ProgressCallback);
        }
        catch (Exception e)
        {
            Console.WriteLine($"Error: {e.Message}");
        }

        Console.WriteLine("Function completed.");
    }
}
