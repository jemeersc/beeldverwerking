#include <cstdio>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

using namespace cv;
using namespace std;

const string FOLDER = "data\\";
const string OUT_FOLDER = "output\\";
const string IN_FOLDER = "input\\";
const int sec = 30;		// seconds until closure

string output_folder(int labo_nr){
	return FOLDER + "labo" + to_string(labo_nr) + "\\" + OUT_FOLDER;
}

string input_folder(int labo_nr){
	return FOLDER + "labo" + to_string(labo_nr) + "\\" + IN_FOLDER;
}

string output_file(int labo_nr, string filename){
	return output_folder(labo_nr) + filename;
}

string input_file(int labo_nr, string filename){
	return input_folder(labo_nr) + filename;
}

Mat open_file(int labo_nr, string filename){
	cout << "Opening image from file: " + input_file(labo_nr, filename) << endl;
	return imread(input_file(labo_nr, filename));
}

Mat open_file_absolute(string filename){
	cout << "Opening image from file: " + filename << endl;
	return imread(filename);
}

bool write_file(int labo_nr, string filename, Mat img){
	cout << "Writing image to file: " + output_file(labo_nr, filename) << endl;
	return imwrite(output_file(labo_nr, filename), img);
}

bool write_file_absolute(string filename, Mat img){
	cout << "Writing image to file: " + filename << endl;
	return imwrite(filename, img);
}

Mat gray(Mat img){
	Mat grey;
	try{
		cvtColor(img, grey, COLOR_RGB2GRAY);
	}
	catch (Exception e){
		cout << "Fout bij omzetten naar grijswaarden" << endl;
	}
	return grey;
}

// LABO 1 - OPGAVE 1:
// Gives the greyscaled and thresholded versions of the image
bool grey_and_threshold(string filename = "clouds.png", bool relative = true){
	const int LABO = 1;
	try{
		Mat img, grey, thres;
		if (relative){
			img = open_file(LABO, filename);
		}
		else{
			img = open_file_absolute(filename);
		}
		if (!img.empty()){
			cvtColor(img, grey, COLOR_RGB2GRAY);
			threshold(grey, thres, 256 / 2, 256, THRESH_TRUNC);
			write_file(LABO,"greyscale_" + filename, grey);
			write_file(LABO, "threshold_50_" + filename, thres);
			//namedWindow("Wolkjes");
			imshow("Wolkjes", img);
			imshow("Grijze Wolkjes", grey);
			imshow("Gethresholde Wolkjes", thres);
			cout << endl << "Press a key to exit, or wait " << sec << " seconds...";
			waitKey(sec * 1000);		// seconds to millis
		}
		return true;
	}
	catch (Exception e){
		cout << "Fout bij laden" << endl;
		return false;
	}

}
// LABO 1 - OPGAVE 1:
// Greyscale and threshold the image
bool labo1(string filename = "clouds.png", bool relative = true){
	return grey_and_threshold(filename,relative);
}
// LABO 1 - OPGAVE 1:
// Greyscale and threshold the image
bool opgave1(string filename = "clouds.png", bool relative = true){
	return grey_and_threshold(filename, relative);
}

const int BLUR_AMOUNT = 31;
// LABO 2 - OPGAVE 2:
// Removes the noise in the image with a certain amount
// --
//	Schrijf een programma dat een PNG afbeelding inleest(bestandsnaam via commandolijn) en deze vervaagt
//	met een Gaussiaans filter. Dit filter vervangt elke pixel door een gewogen gemiddelde van de omliggende
//	pixels.De wegingsfactoren zijn bepaald door een 2D normale verdeling rondom de centrale pixel,
//	dus nabijgelegen pixels hebben meer invloed dan iets verder gelegen pixels.
//	Dit soort filter wordt vaak gebruikt om witte ruis uit het beeld te verwijderen.
//	Witte ruis is een vorm van ruis waarbij elke pixel een willekeurige
//	afwijking ondergaan is van zijn originele waarde.Test je programma op
//	whitenoise.png
//	Toon de originele afbeelding en de vervaagde versie naast elkaar in twee aparte vensters.
//	Nieuwe functies die je nodig hebt :	GaussianBlur
bool remove_noise(int amount = BLUR_AMOUNT, string filename = "whitenoise.png", bool relative = true){

	const int LABO = 2;
	int real_amount;
	if (amount % 2 != 0){
		real_amount = _CMATH_::abs(amount);
	}
	else{
		real_amount = _CMATH_::abs(amount) + 1;
	}
	double sigma = 0.0;

	try{
		Mat img, filtered;
		if (relative){
			img = open_file(LABO, filename);
		}
		else{
			img = open_file_absolute(filename);
		}
		Size size(real_amount, real_amount);
		GaussianBlur(img, filtered, size, sigma);

		if (!img.empty()){
			imshow("Ruisige Wolkjes", img);
			imshow("Gefilterde Wolkjes", filtered);
		}
		write_file(LABO, "filtered_" + filename, filtered);
		cout << endl << "Press a key to exit, or wait " << sec << " seconds...";
		waitKey(sec * 1000);		// seconds to millis
		return true;
	}
	catch (Exception e){
		cout << "Fout bij laden" << endl;
		return false;
	}
}
// LABO 2 - OPGAVE 2:
// Removes the noise in the image with a certain amount
// --
//	Schrijf een programma dat een PNG afbeelding inleest(bestandsnaam via commandolijn) en deze vervaagt
//	met een Gaussiaans filter. Dit filter vervangt elke pixel door een gewogen gemiddelde van de omliggende
//	pixels.De wegingsfactoren zijn bepaald door een 2D normale verdeling rondom de centrale pixel,
//	dus nabijgelegen pixels hebben meer invloed dan iets verder gelegen pixels.
//	Dit soort filter wordt vaak gebruikt om witte ruis uit het beeld te verwijderen.
//	Witte ruis is een vorm van ruis waarbij elke pixel een willekeurige
//	afwijking ondergaan is van zijn originele waarde.Test je programma op
//	whitenoise.png
//	Toon de originele afbeelding en de vervaagde versie naast elkaar in twee aparte vensters.
//	Nieuwe functies die je nodig hebt :	GaussianBlur
bool labo2_1(int amount = BLUR_AMOUNT, string filename = "whitenoise.png", bool relative = true){
	return remove_noise(amount, filename, relative);
}
// LABO 2 - OPGAVE 2:
// Removes the noise in the image with a certain amount
// --
//	Schrijf een programma dat een PNG afbeelding inleest(bestandsnaam via commandolijn) en deze vervaagt
//	met een Gaussiaans filter. Dit filter vervangt elke pixel door een gewogen gemiddelde van de omliggende
//	pixels.De wegingsfactoren zijn bepaald door een 2D normale verdeling rondom de centrale pixel,
//	dus nabijgelegen pixels hebben meer invloed dan iets verder gelegen pixels.
//	Dit soort filter wordt vaak gebruikt om witte ruis uit het beeld te verwijderen.
//	Witte ruis is een vorm van ruis waarbij elke pixel een willekeurige
//	afwijking ondergaan is van zijn originele waarde.Test je programma op
//	whitenoise.png
//	Toon de originele afbeelding en de vervaagde versie naast elkaar in twee aparte vensters.
//	Nieuwe functies die je nodig hebt :	GaussianBlur
bool opgave2(int amount = BLUR_AMOUNT, string filename = "whitenoise.png", bool relative = true){
	return remove_noise(amount, filename, relative);
}

const int SHARP_AMOUNT = 31;
// LABO 2 - OPGAVE 3:
// Unsharpens the image
// --
//	Schrijf een programma dat een PNG afbeelding inleest(bestandsnaam via commandolijn) en hierop
//	unsharp masking toepast.
//	Dit is een techniek om een afbeelding te verscherpen en houdt het volgende in :
//		- vervaag de afbeelding;
//		- bepaal het(absolute) verschil tussen de originele en de vervaagde afbeelding;
//		- tel dit verschil op bij de originele afbeelding.
//	Let erop dat je geen overflow of saturatie krijg in je datatype!
//	Gebruik gepaste schalingsfactoren. Test je programma op unsharp.png
//	Toon het originele beeld en de verscherpte versie naast elkaar.
bool unsharpen(int amount = SHARP_AMOUNT, string filename = "unsharp.png", bool relative = true){

	const int LABO = 2;
	int real_amount;
	if (amount % 2 != 0){
		real_amount = _CMATH_::abs(amount);
	}
	else{
		real_amount = _CMATH_::abs(amount) + 1;
	}
	double sigma = 0.0;

	

	try{
		Mat img, filtered, unsharp;
		Size size(real_amount, real_amount);
		if (relative){
			img = open_file(LABO, filename);
		}
		else{
			img = open_file_absolute(filename);
		}
		GaussianBlur(img, filtered, size, sigma);
		unsharp = img + abs(img - filtered);
		if (!img.empty()){
			imshow("Wazige Hemel", img);
			imshow("Verscherpte Hemel", unsharp);
		}
		else{ cout << "Image is empty!"; }
		write_file(LABO, "unsharpened_" + filename, unsharp);
		cout << endl << "Press a key to exit, or wait " << sec << " seconds...";
		waitKey(sec * 1000);		// seconds to millis
		return true;
	}
	catch (Exception e){
		cout << "Fout bij laden" << endl;
		return false;
	}
}
// LABO 2 - OPGAVE 3:
// Unsharpens the image
// --
//	Schrijf een programma dat een PNG afbeelding inleest(bestandsnaam via commandolijn) en hierop
//	unsharp masking toepast.
//	Dit is een techniek om een afbeelding te verscherpen en houdt het volgende in :
//		- vervaag de afbeelding;
//		- bepaal het(absolute) verschil tussen de originele en de vervaagde afbeelding;
//		- tel dit verschil op bij de originele afbeelding.
//	Let erop dat je geen overflow of saturatie krijg in je datatype!
//	Gebruik gepaste schalingsfactoren. Test je programma op unsharp.png
//	Toon het originele beeld en de verscherpte versie naast elkaar.
bool labo2_2(int amount = SHARP_AMOUNT, string filename = "unsharp.png", bool relative = true){
	return unsharpen(amount, filename, relative);
}
// LABO 2 - OPGAVE 3:
// Unsharpens the image
// --
//	Schrijf een programma dat een PNG afbeelding inleest(bestandsnaam via commandolijn) en hierop
//	unsharp masking toepast.
//	Dit is een techniek om een afbeelding te verscherpen en houdt het volgende in :
//		- vervaag de afbeelding;
//		- bepaal het(absolute) verschil tussen de originele en de vervaagde afbeelding;
//		- tel dit verschil op bij de originele afbeelding.
//	Let erop dat je geen overflow of saturatie krijg in je datatype!
//	Gebruik gepaste schalingsfactoren. Test je programma op unsharp.png
//	Toon het originele beeld en de verscherpte versie naast elkaar.
bool opgave3(int amount = SHARP_AMOUNT, string filename = "unsharp.png", bool relative = true){
	return unsharpen(amount, filename, relative);
}

const int MEDIAN_AMOUNT = 3;
// LABO 2 - OPGAVE 4:
// --
//	Schrijf een programma dat een PNG afbeelding inleest (bestandsnaam via commandolijn) en hieruit de
//	salt and pepper noise verwijdert met een mediaanfilter.
//	Salt and pepper noise is een vorm van ruis waarbij sommige pixels zwart of wit gekleurd worden.
//	Het mediaanfilter vervangt elke pixel door de mediaan van de omliggende.
//	Hierdoor hebben de uitschieters geen invloed meer, in tegenstelling tot een Gaussiaans filter, waar de invloed enkel verminderd wordt.
//	Test je programma op saltandpeppernoise.png
//	Toon het originele beeld en de gefilterde versie naast elkaar.
//	Nieuwe functies die je nodig hebt : medianBlur
bool opgave4(int amount = MEDIAN_AMOUNT, string filename = "saltandpeppernoise.png", bool relative = true){

	const int LABO = 2;

	Mat img, filtered;
	try{
		int real_amount;	// must be odd
		if (amount % 2 != 0){
			real_amount = _CMATH_::abs(amount);
		}
		else{
			real_amount = _CMATH_::abs(amount) + 1;
		}
		if (relative){
			img = open_file(LABO, filename);
		}
		else{
			img = open_file_absolute(filename);
		}
		medianBlur(img, filtered, real_amount);
		if (!img.empty()){
			imshow("Zout en Peperige Hemel", img);
			imshow("Gefilterde Hemel", filtered);
		}
		else{ cout << "Image is empty!"; }
		write_file(LABO, "medianized_" + filename, filtered);
		cout << endl << "Press a key to exit, or wait " << sec << " seconds...";
		waitKey(sec * 1000);		// seconds to millis
		return true;
	}
	catch (Exception e){
		cout << "Fout bij laden" << endl;
		return false;
	}

}
// LABO 2 - OPGAVE 4:
// --
//	Schrijf een programma dat een PNG afbeelding inleest (bestandsnaam via commandolijn) en hieruit de
//	salt and pepper noise verwijdert met een mediaanfilter.
//	Salt and pepper noise is een vorm van ruis waarbij sommige pixels zwart of wit gekleurd worden.
//	Het mediaanfilter vervangt elke pixel door de mediaan van de omliggende.
//	Hierdoor hebben de uitschieters geen invloed meer, in tegenstelling tot een Gaussiaans filter, waar de invloed enkel verminderd wordt.
//	Test je programma op saltandpeppernoise.png
//	Toon het originele beeld en de gefilterde versie naast elkaar.
//	Nieuwe functies die je nodig hebt : medianBlur
bool labo2_3(int amount = MEDIAN_AMOUNT, string filename = "saltandpeppernoise.png", bool relative = true){
	return opgave4(amount, filename, relative);
}
// LABO 2 - OPGAVE 4:
// --
//	Schrijf een programma dat een PNG afbeelding inleest (bestandsnaam via commandolijn) en hieruit de
//	salt and pepper noise verwijdert met een mediaanfilter.
//	Salt and pepper noise is een vorm van ruis waarbij sommige pixels zwart of wit gekleurd worden.
//	Het mediaanfilter vervangt elke pixel door de mediaan van de omliggende.
//	Hierdoor hebben de uitschieters geen invloed meer, in tegenstelling tot een Gaussiaans filter, waar de invloed enkel verminderd wordt.
//	Test je programma op saltandpeppernoise.png
//	Toon het originele beeld en de gefilterde versie naast elkaar.
//	Nieuwe functies die je nodig hebt : medianBlur
bool medianize(int amount = MEDIAN_AMOUNT, string filename = "saltandpeppernoise.png", bool relative = true){
	return opgave4(amount, filename, relative);
}

// LABO 2 - OPGAVE 5:
// --
//	Schrijf een programma dat een PNG afbeelding inleest (bestandsnaam via commandolijn),
//	omzet in grijswaarden en hiervan de horizontale eerste afgeleide berekent.
//	Dit doe je door te filteren met een horizontaal
//	Sobel kernel :
//		|-1	 0	1|
//		|-2	 0	2|
//		|-1	 0	1|
//	Dit geeft je een maat voor de horizontale verandering in een afbeelding.
//	Test je programma op building.png
//	Nieuwe functies:	Sobel
bool opgave5(string filename = "building.png", bool relative = true){

	const int LABO = 2;

	Mat img, grey, filtered;

	int xorder = 1, yorder = 2, ksize = 3;	// nodig voor Sobel-matrix

	try{
		if (relative){
			img = open_file(LABO, filename);
		}
		else{
			img = open_file_absolute(filename);
		}
		grey = gray(img);
		Sobel(grey, filtered, grey.depth(),xorder,yorder, ksize);
		if (!img.empty()){
			imshow("Gebouw", img);
			imshow("Grijs Gebouw", grey);
			imshow("Sobel Gebouw", filtered);
		}
		else{ cout << "Image is empty!"; }
		write_file(LABO, "sobel_" + filename, filtered);
		cout << endl << "Press a key to exit, or wait " << sec << " seconds...";
		waitKey(sec * 1000);		// seconds to millis
		return true;
	}
	catch (Exception e){
		cout << "Fout bij laden" << endl;
		return false;
	}
}
// LABO 2 - OPGAVE 5:
// --
//	Schrijf een programma dat een PNG afbeelding inleest (bestandsnaam via commandolijn),
//	omzet in grijswaarden en hiervan de horizontale eerste afgeleide berekent.
//	Dit doe je door te filteren met een horizontaal
//	Sobel kernel :
//		|-1	 0	1|
//		|-2	 0	2|
//		|-1	 0	1|
//	Dit geeft je een maat voor de horizontale verandering in een afbeelding.
//	Test je programma op building.png
//	Nieuwe functies:	Sobel
bool sobel_randen(string filename = "building.png", bool relative = true){
	return opgave5(filename, relative);
}
// LABO 2 - OPGAVE 5:
// --
//	Schrijf een programma dat een PNG afbeelding inleest (bestandsnaam via commandolijn),
//	omzet in grijswaarden en hiervan de horizontale eerste afgeleide berekent.
//	Dit doe je door te filteren met een horizontaal
//	Sobel kernel :
//		|-1	 0	1|
//		|-2	 0	2|
//		|-1	 0	1|
//	Dit geeft je een maat voor de horizontale verandering in een afbeelding.
//	Test je programma op building.png
//	Nieuwe functies:	Sobel
bool labo2_4(string filename = "building.png", bool relative = true){
	return opgave5(filename, relative);
}

// LABO 2 - OPGAVE 6:
// --
//	Schrijf een programma dat een PNG afbeelding inleest (bestandsnaam via commandolijn) en deze filtert met
//	een 15x15 kernel waarvan de eerste 7 diagonaalelementen de waarde 1/7 hebben, alle andere zijn nul.
//	Zorg dat het ankerpunt van de kernel (de pixel waarin de berekende waarde terecht komt) rechts onderaan
//	ligt in de kernel in plaats van in het midden zoals gebruikelijk.
//	Wat verwacht je dat dit filter zal doen ?
//	Test je programma op blots.png
//	Toon het origineel en het gefilterde beeld naast elkaar.
//	Nieuwe functies :	filter2D
bool opgave6(string filename = "blots.png", bool relative = true){

	const int LABO = 2;

	Mat img, filtered;

	const int KERNEL_SIZE = 15;
	double matrix[KERNEL_SIZE][KERNEL_SIZE];
	for (int i = 0; i < KERNEL_SIZE; i++){
		for (int j = 0; j < KERNEL_SIZE; j++){
			if (i == j && i < 7){
				matrix[i][j] = 1.0/7;
			}
			else{
				matrix[i][j] = 0.0;
			}
		}
	}

	try{
		if (relative){
			img = open_file(LABO, filename);
		}
		else{
			img = open_file_absolute(filename);
		}

		Mat kernel(KERNEL_SIZE, KERNEL_SIZE, img.depth(), matrix);
		Point anchor(kernel.cols - 1, kernel.rows - 1);
		cout << anchor << endl;
		filter2D(img, filtered, img.depth(), kernel, anchor);

		if (!img.empty()){
			imshow("Blots", img);
			imshow("Geefilterde Blots", filtered);
		}
		else{ cout << "Image is empty!"; }
		write_file(LABO, "filtered_" + filename, filtered);
		cout << endl << "Press a key to exit, or wait " << sec << " seconds...";
		waitKey(sec * 1000);		// seconds to millis
		return true;
	}
	catch (Exception e){
		cout << "Fout bij laden" << endl;
		return false;
	}
}
// LABO 2 - OPGAVE 6:
// --
//	Schrijf een programma dat een PNG afbeelding inleest (bestandsnaam via commandolijn) en deze filtert met
//	een 15x15 kernel waarvan de eerste 7 diagonaalelementen de waarde 1/7 hebben, alle andere zijn nul.
//	Zorg dat het ankerpunt van de kernel (de pixel waarin de berekende waarde terecht komt) rechts onderaan
//	ligt in de kernel in plaats van in het midden zoals gebruikelijk.
//	Wat verwacht je dat dit filter zal doen ?
//	Test je programma op blots.png
//	Toon het origineel en het gefilterde beeld naast elkaar.
//	Nieuwe functies :	filter2D
bool filter_blots(string filename = "blots.png", bool relative = true){
	return opgave6(filename, relative);
}


// LABO 3 - OPGAVE 7:
// --
//	Schrijf een programma dat de afbeelding rainbowdiscs.png omzet zoals in onderstaande figuur.
//	Om hard gecodeerde paden te vermijden, wordt de bestandsnaam nog steeds via de commandolijn meegegeven.
//	Nieuwe functies :	erode, dilate, getStructuringElement
//
//3 kernel shapes:
//Rectangular box : MORPH_RECT
//Cross : MORPH_CROSS
//Ellipse : MORPH_ELLIPSE
bool opgave7(int erosion_width = 3, int erosion_height = 3, int dilation_width = 1, int dilation_height = 7, int kernel_shape = MORPH_RECT, string filename = "rainbowdiscs.png", bool relative = true){

	const int LABO = 3;

	Mat img, eroded, dilated, opened, closed;

	Mat erosion_kernel = getStructuringElement(kernel_shape,
		Size(2 * erosion_width + 1, 2 * erosion_height + 1),
		Point(erosion_width, erosion_height));

	Mat dilation_kernel = getStructuringElement(kernel_shape,
		Size(2 * dilation_width + 1, 2 * dilation_height + 1),
		Point(dilation_width, dilation_height));

	try{
		if (relative){
			img = open_file(LABO, filename);
		}
		else{
			img = open_file_absolute(filename);
		}

		erode(img, eroded, erosion_kernel);
		dilate(img, dilated, dilation_kernel);
		dilate(eroded, opened, dilation_kernel);
		erode(dilated, closed, erosion_kernel);

		if (!img.empty()){
			imshow("Rainbow Dots", img);
			imshow("Erosie bij Rainbow Dots", eroded);
			imshow("Dilatie bij Rainbow Dots", dilated);
			imshow("Opening bij Rainbow Dots", opened);
			imshow("Sluiting bij Rainbow Dots", closed);
		}
		else{ cout << "Image is empty!"; }
		write_file(LABO, "eroded_" + filename, eroded);
		write_file(LABO, "dilated_" + filename, dilated);
		write_file(LABO, "opened_" + filename, opened);
		write_file(LABO, "closed_" + filename, closed);
		cout << endl << "Press a key to exit, or wait " << sec << " seconds...";
		waitKey(sec * 1000);		// seconds to millis
		return true;
	}
	catch (Exception e){
		cout << "Fout bij laden" << endl;
		return false;
	}
}

// LABO 4 - OPGAVE 8:
// --
//	Schrijf een programma dat de schaduw van de fotograaf in shadow.png verticaal trekt met een sheartransformatie.
//	De transformatiematrix om horizontaal te shearen heeft deze vorm :
//		|	1	m	0	|
//		|	0	1	0	|
//	waarbij m de shear factor is.
//	Je kan translatie toevoegen door pixel ofsets in de derde kolom te plaatsen.
//	Nieuwe functies :	warpAffine
bool opgave8(float shear_factor = -0.125, float dx = 80, float dy = 0, string filename = "shadow.png", bool relative = true){

	const int LABO = 4;

	float shear[2][3] = { { 1, shear_factor, dx }, { 0, 1, dy } };

	Mat img, warped;

	try{
		if (relative){
			img = open_file(LABO, filename);
		}
		else{
			img = open_file_absolute(filename);
		}

		Mat kernel(2, 3, CV_32F/*img.depth()*/, shear);
		Size dsize = img.size();
		warped = Mat::zeros(img.rows, img.cols, img.type());
		warpAffine(img, warped, kernel, warped.size());
		img.copySize(warped);
		if (!img.empty()){
			imshow("Shadow", img);
			imshow("Warped Shadow", warped);
		}
		else{ cout << "Image is empty!"; }
		write_file(LABO, "warped_" + filename, warped);
		cout << endl << "Press a key to exit, or wait " << sec << " seconds...";
		waitKey(sec * 1000);		// seconds to millis
		return true;
	}
	catch (Exception e){
		cout << "Fout bij laden" << endl;
		return false;
	}
}


Point2f res[4];
Point2f points[4];
int index;
static void onMouse(int event, int x, int y, int d, void* ptr){
	if (event != EVENT_LBUTTONDOWN)
		return;
	Point2f*p = (Point2f*)ptr;
	p->x = x;
	p->y = y;
	points[index] = *p;
	index++;
}
// LABO 4 - OPGAVE 9:
// --
//	Schrijf een programma dat ervoor zorgt dat de fotograaf in shadow_box.png
//	niet enkel verticaal komt te staan, maar ook proportioneel klopt.
//	In je programma klik je de 4 hoekpunten aan van de vierhoek die je wil omvormen tot een rechthoek,
//	waarna de juiste perspectieftransformatie gezocht en uitgevoerd wordt.
//	Functies die je nodig hebt :	setMouseCallback, getPerspectiveTransform, warpPerspective
bool opgave9(float shear_factor = -0.125, float dx = 80, float dy = 0, string filename = "shadow_box.png", bool relative = true){

	const int LABO = 4;

	Mat img, warped;

	try{
		if (relative){
			img = open_file(LABO, filename);
		}
		else{
			img = open_file_absolute(filename);
		}

		Size dsize = img.size();
		warped = Mat::zeros(img.rows, img.cols, img.type());
		img.copySize(warped);
		if (!img.empty()){
				namedWindow("Shadow");
				Point2f *p = new Point2f;
				setMouseCallback("Shadow", onMouse, p);
				cout << "Select points in this order:" << endl << " top left ;  top right ; bottom left ; bottom right" << endl;
				imshow("Shadow", img);
				while (index < 4){
					waitKey(200);		// seconds to millis
				}
				destroyWindow("Shadow");
			res[0] = Point2f(points[2].x, points[0].y);		// top left
			res[1] = Point2f(points[3].x, points[0].y);		// top right
			res[2] = points[2];								// bottom left
			res[3] = Point2f(points[3].x, points[2].y);		// bottom right
			Mat kernel = getPerspectiveTransform(points, res);
			warpPerspective(img, warped, kernel, warped.size());
			imshow("Framed Shadow", img);
			imshow("Warped Shadow", warped);
		}
		else{ cout << "Image is empty!"; }
		write_file(LABO, "warped_" + filename, warped);
		cout << endl << "Press a key to exit, or wait " << sec << " seconds...";
		waitKey(sec * 1000);		// seconds to millis
		return true;
	}
	catch (Exception e){
		cout << "Fout bij laden" << endl;
		return false;
	}
}

// LABO 5 - OPGAVE 10:
// --
//	Schrijf een programma dat de randen van de gele stroken in rays.png extraheert.
//	De naam van de afbeelding is nog steeds een commandolijnparameter.
//	Je kan dit doen als volgt:
//	- maak een 1D Gaussiaanse kernel met de functie getGaussianKernel
//	- kopieer deze in de middenste kolom van een vierkante matrix met Mat::col en Mat::copyTo
//	- maak nog een 1D Gaussiaanse kernel met kleinere standaarddeviatie en transponeer deze(Mat::t) om een rijmatrix te bekomen
//	- filter de vierkante matrix(die de kolomkernel bevat) met deze rijkernel om een 2D Gaussiaan te bekomen
//	- leidt deze horizontaal of verticaal af met Sobel om een DoG filter(differential of Gaussian) te bekomen;
//	- maak een rotatiematrix voor de juiste hoek van de gele stroken(ca. 75 graden) met getRotationMatrix2D
//	- roteer je DoG filter met deze rotatiematrix(warpAffine)
//	- zet de afbeelding om naar grijswaarden en filter ze met het geroteerde DoG filter;
//	- neem de absolute waarde van de filterrespons met abs
//	- tweak de parameters van de Gaussianen en de kernelgrootte zodat je een goede orientatieselectiviteit verkrijgt;
bool opgave10(float shear_factor = -0.125, float dx = 80, float dy = 0, string filename = "rays.png", bool relative = true){

	const int LABO = 5;

	Mat img, warped;

	try{
		if (relative){
			img = open_file(LABO, filename);
		}
		else{
			img = open_file_absolute(filename);
		}

		Size dsize = img.size();
		warped = Mat::zeros(img.rows, img.cols, img.type());
		img.copySize(warped);
		if (!img.empty()){
			namedWindow("Shadow");
			Point2f *p = new Point2f;
			setMouseCallback("Shadow", onMouse, p);
			cout << "Select points in this order:" << endl << " top left ;  top right ; bottom left ; bottom right" << endl;
			imshow("Shadow", img);
			while (index < 4){
				waitKey(200);		// seconds to millis
			}
			destroyWindow("Shadow");
			res[0] = Point2f(points[2].x, points[0].y);		// top left
			res[1] = Point2f(points[3].x, points[0].y);		// top right
			res[2] = points[2];								// bottom left
			res[3] = Point2f(points[3].x, points[2].y);		// bottom right
			Mat kernel = getPerspectiveTransform(points, res);
			warpPerspective(img, warped, kernel, warped.size());
			imshow("Framed Shadow", img);
			imshow("Warped Shadow", warped);
		}
		else{ cout << "Image is empty!"; }
		write_file(LABO, "warped_" + filename, warped);
		cout << endl << "Press a key to exit, or wait " << sec << " seconds...";
		waitKey(sec * 1000);		// seconds to millis
		return true;
	}
	catch (Exception e){
		cout << "Fout bij laden" << endl;
		return false;
	}
}


//
//int main(int argc, char **argv){
//	bool correct =
//		// Vul hier naam te gebruiken opdracht in:
//		opgave1();
//		// ---------------------------------------
//
//	if (!correct){
//		cout << endl << "Press a key to exit...";
//		cin.get();
//	}
//
//	return 0;
//}
