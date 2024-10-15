


//向文件写入数组数据
//参数1：文件名称，参数2：文件输出模式，参数3：数组，参数4：数组长度，参数5：表头描述
void exel_output(string name, std::ios_base::openmode my_mode, float a[], int j, string discription)
{
	ofstream outfile(name, my_mode);
	if (!outfile)
	{
		cerr << "open error!" << endl;
		exit(1);
	}
	cout << name << " ok" << endl;
	outfile << discription << "\t"; 	//建立表头描述
	for (int i = 0; i < j; i++)
	{
		outfile << a[i] << "\t";			//把数组所有数据写在这一行
	}
	outfile << "\n";					//指针指到下一行
	outfile.close();
}

//C语言输出文件的方法////////////////////////////////////////////////////////////////////////////
//void writeExcel(void ) {
//	
//	FILE *fp = NULL;
//	int t;
//	char ch;
// 
//	fp = fopen("E:\\ALG_TEST_FILE\\test.xls", "w");
//	
//	for (int i = 0; i < 2; i++) {
//		printf("please input:");
//		scanf("%d %c", &t, &ch);
//		fprintf(fp, "%d\t%c\n", t, ch);
//	}
//	fclose(fp);
// 
//}

void file_create(string name, std::ios_base::openmode my_mode)//建立文件
{
	ofstream outfile(name, my_mode);
	if (!outfile)
	{
		cerr << "open error!" << endl;
		exit(1);
	}
	cout << name << " ok" << endl;
	outfile.close();
}