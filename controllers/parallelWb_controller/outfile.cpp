


//���ļ�д����������
//����1���ļ����ƣ�����2���ļ����ģʽ������3�����飬����4�����鳤�ȣ�����5����ͷ����
void exel_output(string name, std::ios_base::openmode my_mode, float a[], int j, string discription)
{
	ofstream outfile(name, my_mode);
	if (!outfile)
	{
		cerr << "open error!" << endl;
		exit(1);
	}
	cout << name << " ok" << endl;
	outfile << discription << "\t"; 	//������ͷ����
	for (int i = 0; i < j; i++)
	{
		outfile << a[i] << "\t";			//��������������д����һ��
	}
	outfile << "\n";					//ָ��ָ����һ��
	outfile.close();
}

//C��������ļ��ķ���////////////////////////////////////////////////////////////////////////////
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

void file_create(string name, std::ios_base::openmode my_mode)//�����ļ�
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