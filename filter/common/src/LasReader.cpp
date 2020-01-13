#include "LasReader.hpp"

/*****************
    LAS1.3文件格式
Item									 Format				  Size
------------------------------------------------------------------------------------------
File Signature (“LASF”)  				char[4] 			4 bytes
File Source  ID							unsigned short		2 bytes
Global Encoding							unsigned short		2 bytes
Project ID - GUID data 1				unsigned long		4 bytes
Project ID - GUID data 2				unsigned short		2 bytes
Project ID - GUID data 3				unsigned short		2 bytes
Project ID - GUID data 4				unsigned char[8]    8 bytes
Version Major  							unsigned char  		1 byte  
Version Minor  							unsigned char  		1 byte  
System Identifier 						char[32]  			32 bytes
Generating Software  					char[32]  			32 bytes
File Creation Day of Year  				unsigned short  	2 bytes
File Creation Day of Year 				unsigned short  	2 bytes
Header Size 							unsigned short 		2 bytes
Offset to point data 					unsigned long 		4 bytes
Number of Variable Length Records		unsigned long 		4 bytes
Point Data Format ID (0-99 for spec) 	unsigned char 		1 byte
Point Data Record Length 				unsigned short 		2 bytes
Number of point records 				unsigned long 		4 bytes
Number of points by return 				unsigned long[7] 	28 bytes
X scale factor							Double 				8 bytes
Y scale factor 							Double 				8 bytes
Z scale factor 							Double 				8 bytes
X offset 								Double 				8 bytes
Y offset 								Double 				8 bytes
Z offset 								Double 				8 bytes
Max X 									Double 				8 bytes
Min X 									Double 				8 bytes
Max Y 									Double 				8 bytes
Min Y 									Double 				8 bytes
Max Z 									Double 				8 bytes
Min Z 									Double 				8 bytes
Start of Waveform Data Packet Record 	Unsigned long long 	8 bytes

与las 1.2的兼容性：
公共头文件区有一个不可避免的改变，就是添加了波形数据起始位置。该超长整形数据被添加到了公共头文件区的末尾，
所以对于不需要波形数据的las 1.2点读器来说，只需要一点或者不需要改变。
对于点数据记录类型0到3，没有任何改变。而在点数据记录类型4和5中加入了波形数据。
**************/

LasReader::LasReader():
    InputFile(NULL),
    fname_envParameteres(NULL)
{}

LasReader::LasReader(const char *input_file_name):
    InputFile(NULL),
    fname_envParameteres(NULL)
{
    int i = strlen(input_file_name);
    if (i > 0 && input_file_name != NULL)
    {
        fname_envParameteres = new char[i+1];
        strcpy(fname_envParameteres,input_file_name);
        fname_envParameteres[i] = '\0';
    }
}

LasReader::~LasReader()
{
    if(fname_envParameteres != NULL)
    {
        delete[] fname_envParameteres;
        fname_envParameteres = NULL;
    }
}

LasReader::file_header_type::file_header_type():
    OffsetToData(0),
    DataFormat(NULL),
    Num_p(0),
    x_s(0.0),
	y_s(0.0),
	z_s(0.0),
	x_offset(0.0),
	y_offset(0.0),
	z_offset(0.0),
	max_x(0.0),
	min_x(0.0),
	max_y(0.0),
	min_y(0.0),
	max_z(0.0),
	min_z(0.0)
{}

LasReader::file_header_type::file_header_type(const LasReader::file_header_type &s) :
	OffsetToData(s.OffsetToData),
	DataFormat(s.DataFormat),
	Num_p(s.Num_p),
	x_s(s.x_s),
	y_s(s.y_s),
	z_s(s.z_s),
	x_offset(s.x_offset),
	y_offset(s.y_offset),
	z_offset(s.z_offset),
	max_x(s.max_x),
	min_x(s.min_x),
	max_y(s.max_y),
	min_y(s.min_y),
	max_z(s.max_z),
	min_z(s.min_z)
{}

void LasReader::file_header_type::clear()
{
    OffsetToData = 0;
	DataFormat = NULL;
	Num_p = 0;
	x_s = 0.0;
	y_s = 0.0;
	z_s = 0.0;
	x_offset = 0.0;
	y_offset = 0.0;
	z_offset = 0.0;
	max_x = 0.0;
	min_x = 0.0;
	max_y = 0.0;
	min_y = 0.0;
	max_z = 0.0;
	min_z = 0.0;
}

bool LasReader::SetFileName(const char *input_file_name)
{
    int i = strlen(input_file_name);
    if(i > 0 && fname_envParameteres == NULL)
    {
        fname_envParameteres = new char[i+1];
        strcpy(fname_envParameteres,input_file_name);
        fname_envParameteres[i] = '\0';
    }
    return true;
}

bool LasReader::getEnvFileName(char *strCopied)
{
	int str_len = strlen(fname_envParameteres) + 1;

	if (str_len > 1)
	{
		strCopied = new char[str_len];
		strcpy(strCopied, fname_envParameteres);
		strCopied[str_len - 1] = '\0';
	}
	return true;
}

bool LasReader::ReadHeader(){
	InputFile = fopen(fname_envParameteres,"rb");
	// Read the offset to point data:
	fseek(InputFile,96,0);//函数设置文件指针InputFile的位置,偏移96个字节
	unsigned long *offset = &file_header.OffsetToData;
	fread(offset,4,1,InputFile);
	// Read the data format ID:
	fseek(InputFile,4,1);
	unsigned char *format = &file_header.DataFormat;
	fread(format,1,1,InputFile);
	// Read the number of point records:
	fseek(InputFile,2,1);
	unsigned long* num_points = &file_header.Num_p;
	fread(num_points,4,1,InputFile);

	// Read the scale factors:
	fseek(InputFile,20,1);
	double* x_d;
	x_d = &file_header.x_s;
	fread(x_d, 8, 1, InputFile);
	// Fread automatically moves the file pointer.
	x_d = &file_header.y_s;
	fread(x_d, 8, 1, InputFile);
	x_d = &file_header.z_s;
	fread(x_d, 8, 1, InputFile);
	x_d = &file_header.x_offset;
	fread(x_d, 8, 1, InputFile);
	x_d = &file_header.y_offset;
	fread(x_d, 8, 1, InputFile);
	x_d = &file_header.z_offset;
	fread(x_d, 8, 1, InputFile);
	x_d = &file_header.max_x;
	fread(x_d, 8, 1, InputFile);
	x_d = &file_header.min_x;
	fread(x_d, 8, 1, InputFile);
	x_d = &file_header.max_y;
	fread(x_d, 8, 1, InputFile);
	x_d = &file_header.min_y;
	fread(x_d, 8, 1, InputFile);
	x_d = &file_header.max_z;
	fread(x_d, 8, 1, InputFile);
	x_d = &file_header.min_z;
	fread(x_d, 8, 1, InputFile);

	fflush(InputFile);
	fclose(InputFile);
	return 0;
}

//Retrieves the header information
bool LasReader::GetHeader(LasReader::file_header_type* header)
{
	header->OffsetToData = file_header.OffsetToData;
	header->DataFormat = file_header.DataFormat;
	header->Num_p = file_header.Num_p;
	header->x_s = file_header.x_s;
	header->y_s = file_header.y_s;
	header->z_s = file_header.z_s;
	header->x_offset = file_header.x_offset;
	header->y_offset = file_header.y_offset;
	header->z_offset = file_header.z_offset;
	header->max_x = file_header.max_x;
	header->min_x = file_header.min_x;
	header->max_y = file_header.max_y;
	header->min_y = file_header.min_y;
	header->max_z = file_header.max_z;
	header->min_z = file_header.min_z;

	return true;

}